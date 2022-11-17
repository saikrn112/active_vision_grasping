#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h> // for compute3DCentroid
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/common/projection_matrix.h>
#include <Eigen/Core>
#include <math.h>
#include <array>
#include <vector>


using std::placeholders::_1;
using Eigen::placeholders::all;

class NormalProcessor : public rclcpp::Node
{

public:
    NormalProcessor(): Node("pc_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( "objectPoints", 
                                                                                10, 
                                                                                std::bind(&NormalProcessor::topic_callback, 
                                                                                this, _1));

      grasp_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("graspPoints", 10);
    }


private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grasp_points_pub_;


  // Collinearty check function
  bool isCollinear(const Eigen::Vector3f& vec1, const Eigen::Vector3f& vec2, double eps = 0.1) const
  {
    // Normalize the functions before calculation dot product
    const auto& vec1_norm = vec1.normalized();
    const auto& vec2_norm = vec2.normalized();

    // derived dot product
    auto dot_product_val = vec1_norm.dot(vec2_norm);

    // making sure that dot product value is around -1 (angle as 180)
    //  since there can be numerical accuracies not giving us perfect -1
    if ((-1 - eps <= dot_product_val) && (dot_product_val <= -1 + eps))
    {
      return true;
    } 
    return false;
  }

  struct PointPairHasher
  {
    template <typename T1, typename T2>
    std::size_t operator() (const std::pair<T1, T2> &pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
  };

 /* Function to calculate the best grasp contact pairs in the segmented point cloud */
 std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> 
    getBestGraspContactPair(const Eigen::Matrix3Xf& normals, 
                            const Eigen::Matrix3Xf& contact_points, 
                            const Eigen::Vector3f& centroid) const
  {
    // Initialize the containers to store the contact pairs
    std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f>> cp_pairs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr grasp_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // ideal best grasp angle value
    double best_grasp_angle = 0;

    // define angle threshold
    float angle_threshold_degree = 10;
    float angle_threshold = angle_threshold_degree * (M_PI / 180);

    // Matrix of Vectors between contact points and centroid
    auto CX0 = contact_points.colwise() - centroid;

    /* Check against all the contact points iteratively */
    for (size_t i=0; i < normals.cols(); i++)
    {
      // 1st contact point that is considered
      const auto& C1 = contact_points(all, i);

      // 1st contact point normal
      const auto& C1N = normals(all, i);
      
      // vector between 1st contact point and centroid
      const auto& C10 = CX0(all, i);

      for (size_t j=0; j<normals.cols(); j++)
      {
        // exclude comparing between same contact points
        if (i==j)
        {
          continue;
        }

        // 2nd contact point that is considered
        const auto& C2 = contact_points(all, j);

        // vector between 2nd contact point and centroid
        const auto& C20 = CX0(all, j);

        // check if vectors between 
        //      (1st contact point and centroid) 
        //  and (2nd contact point and centroid) 
        //  are collinear
        auto is_collinear = isCollinear(C10,C20);

        // if they are collinear check if they satisfy the necessary 
        //  force vector grasp formulation
        if (is_collinear)
        {

          // 1st contact point normal
          const auto& C2N = normals(all, j);

          // vector between contact points
          auto C1C2 = (C1-C2).normalized();

          // calculate angles between contact points and corresponding normals
          auto angle1 = acos(C1N.dot(C1C2));
          auto angle2 = acos(C2N.dot(C1C2));
          double grasp_angle = angle1 + angle2;

          // Check if corresponding grasp angle is falling within the threshold
          if(M_PI - angle_threshold < grasp_angle && grasp_angle < M_PI + angle_threshold)
          {
             // Check if the corresponding grasp angle is better
             // than previous candidate grasp angles
             if (grasp_angle >= best_grasp_angle)
             {
                grasp_point_cloud->push_back(pcl::PointXYZ(C1(0),C1(1),C1(2)));
                grasp_point_cloud->push_back(pcl::PointXYZ(C2(0),C2(1),C2(2)));
                cp_pairs.push_back({C1, C2});
                best_grasp_angle = grasp_angle;
             }
          }
        }
      }
    }

    auto output_grasp_points = new sensor_msgs::msg::PointCloud2;
    pcl::PCLPointCloud2::Ptr cloud_grasp_points(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*grasp_point_cloud,*cloud_grasp_points);
    pcl_conversions::fromPCL(*cloud_grasp_points, *output_grasp_points);
    output_grasp_points->header.frame_id = "camera_link";
    grasp_points_pub_->publish(*output_grasp_points);

    return cp_pairs;
    }    

    pcl::PointCloud<pcl::PointXYZ>::Ptr sensorMsgToXYZcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      // Convert sensor_msg::PointCloud2 to pcl::PCLPointCloud2
      pcl::PCLPointCloud2::Ptr msg_cloudPtr(new pcl::PCLPointCloud2); // container for pcl::PCLPointCloud
      pcl_conversions::toPCL(*msg, *msg_cloudPtr); // convert to PCLPointCloud2 data type

      // Convert pcl::PCLPointCloud2 to PointXYZ data type
      pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*msg_cloudPtr,*XYZcloudPtr);
      return XYZcloudPtr;
    }

    
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloudPtr = sensorMsgToXYZcloud(msg);

      // 1. Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (input_cloudPtr);

      // Create an empty KDTree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);

      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr normals_cloud (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.005);
      ne.useSensorOriginAsViewPoint();

      // 2. Compute the features
      ne.compute (*normals_cloud);
      RCLCPP_INFO_STREAM(this->get_logger(), "# of normals: " << normals_cloud->size ());

      // 3. Compute centroid
      Eigen::Vector4f centroid_vector;
      pcl::compute3DCentroid (*input_cloudPtr, centroid_vector);   
      pcl::PointXYZ centroid_pointxyz(centroid_vector[0], centroid_vector[1], centroid_vector[2]);

      // 4. Flip normals towards centroid
      Eigen::Matrix3Xf normal_matrix(3, normals_cloud->size());
      Eigen::Matrix3Xf cloud_matrix(3,normals_cloud->size());
      for(size_t i = 0; i < normals_cloud->size(); i++) 
      {
        Eigen::Vector3f normal_vector = normals_cloud->at(i).getNormalVector4fMap().head(3);

        pcl::flipNormalTowardsViewpoint(input_cloudPtr->at(i), centroid_vector[0], centroid_vector[1], centroid_vector[2], normal_vector); // flipping normal towards viewpoint
        normal_matrix(0,i) = normal_vector[0];  // storing each normal vector in the matrix
        normal_matrix(1,i) = normal_vector[1];
        normal_matrix(2,i) = normal_vector[2];

        cloud_matrix(0,i) = input_cloudPtr->points[i].x;   // storing each cloud coordinate in the matrix
        cloud_matrix(1,i) = input_cloudPtr->points[i].y;
        cloud_matrix(2,i) = input_cloudPtr->points[i].z;
      }

      const auto& data = getBestGraspContactPair(normal_matrix, cloud_matrix, centroid_vector.head(3));
      RCLCPP_INFO_STREAM(this->get_logger(), "Size of data: " << data.size());
      
    };
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NormalProcessor>());
  rclcpp::shutdown();
  return 0;
}
