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
#include <pcl/visualization/cloud_viewer.h>
//#include <visualization_msgs/Marker.h>
#include <pcl/common/projection_matrix.h>
#include <Eigen/Core>
#include <math.h>
#include <array>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

using std::placeholders::_1;
using Eigen::placeholders::all;

class PointCloudProcessor : public rclcpp::Node
{

public:
    PointCloudProcessor(): Node("pc_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( "realsense/points", 
                                                                                10, 
                                                                                std::bind(&PointCloudProcessor::topic_callback, 
                                                                                this, _1));

      segmented_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("objectPoints", 10);
      table_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tablePoints", 10);
      grasp_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("graspPoints", 10);
    }


private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmented_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr table_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grasp_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr centroid_pub_;

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
    
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      // Convert sensor_msg::PointCloud2 to pcl::PCLPointCloud2
      pcl::PCLPointCloud2::Ptr cloudPtr(new pcl::PCLPointCloud2); // container for pcl::PCLPointCloud
      pcl_conversions::toPCL(*msg, *cloudPtr); // convert to PCLPointCloud2 data type

      // 1. Downsample 
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloudPtr);
      sor.setLeafSize (0.008f, 0.008f, 0.008f);
      sor.filter (*cloudPtr);

      // Convert pcl::PCLPointCloud2 to PointXYZ data type
      pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*cloudPtr,*XYZcloudPtr);

      // 2. Distance Thresholding: Filter out points that are too far away, e.g. the floor
      auto plength = XYZcloudPtr->size();   // Size of the point cloud
      pcl::PointIndices::Ptr farpoints(new pcl::PointIndices());  // Container for the indices
      for (int p = 0; p < plength; p++)
      {
          // Calculate the distance from the origin/camera
          float distance = (XYZcloudPtr->points[p].x * XYZcloudPtr->points[p].x) +
                           (XYZcloudPtr->points[p].y * XYZcloudPtr->points[p].y) + 
                           (XYZcloudPtr->points[p].z * XYZcloudPtr->points[p].z);
          
          if (distance > 1) // Threshold = 1
          {
            farpoints->indices.push_back(p);    // Store the points that should be filtered out
          }
      }

      // 3. Extract the filtered point cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(XYZcloudPtr);
      extract.setIndices(farpoints);          // Filter out the far points
      extract.setNegative(true);
      extract.filter(*XYZcloudPtr);

      // 4. RANSAC; Plane model segmentation from pcl
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      // 5. Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);
      seg.setInputCloud (XYZcloudPtr);
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size () == 0)
      {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
      }

      // 6. Extract the inliers
      pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloud_filtered(new pcl::PointCloud<pcl::PointXYZ>); // container for pcl::PointXYZ
      pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloud_filtered_table(new pcl::PointCloud<pcl::PointXYZ>); // container for pcl::PointXYZ
      extract.setInputCloud (XYZcloudPtr);
      extract.setIndices (inliers);
      extract.setNegative (false);  // false -> major plane, true -> object
      extract.filter (*XYZcloud_filtered_table);

      extract.setInputCloud (XYZcloudPtr);
      extract.setIndices (inliers);
      extract.setNegative (true);  // false -> major plane, true -> object
      extract.filter (*XYZcloud_filtered);

      //pcl::io::savePCDFileASCII("firstcloud.pcd", *XYZcloud_filtered);  

      // // 7. NORMAL ESTIMATION
      // // Create the normal estimation class, and pass the input dataset to it
      // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      // ne.setInputCloud (XYZcloud_filtered);

      // // Create an empty KDTree representation, and pass it to the normal estimation object.
      // // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      // ne.setSearchMethod (tree);

      // // Output datasets
      // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

      // // Use all neighbors in a sphere of radius 3cm
      // ne.setRadiusSearch (0.005);
      // ne.useSensorOriginAsViewPoint();

      // // 7.1 Compute the features
      // ne.compute (*cloud_normals);
      // RCLCPP_INFO_STREAM(this->get_logger(), "# of normals: " << cloud_normals->size ());

      // 8. CENTROID
      // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
      //Eigen::Vector4f xyz_centroid;
      
      // 9. Estimate the XYZ centroid
      //pcl::compute3DCentroid (*XYZcloud_filtered, xyz_centroid);
      // auto centroid_point = new sensor_msgs::msg::PointCloud2;                   
      // pcl::PCLPointCloud2::Ptr centroid_cloud_point(new pcl::PCLPointCloud2);  
      // pcl::PointCloud<pcl::PointXYZ> centroid_point_cloud;
      // centroid_point_cloud.push_back(pcl::PointXYZ(xyz_centroid(0),xyz_centroid(1),xyz_centroid(2))); 
      // pcl::toPCLPointCloud2(centroid_point_cloud,*centroid_cloud_point);   
      // pcl_conversions::fromPCL(*centroid_cloud_point, *centroid_point);          
      // centroid_pub_->publish(*centroid_point);


       // Table
      //XYZcloud_filtered_table->push_back(pcl::PointXYZ(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]));
      auto output_table = new sensor_msgs::msg::PointCloud2;                  // TABLE: container for sensor_msgs::msg::PointCloud2
      pcl::PCLPointCloud2::Ptr cloud_filtered_table(new pcl::PCLPointCloud2); // TABLE: container for pcl::PCLPointCloud2
      pcl::toPCLPointCloud2(*XYZcloud_filtered_table,*cloud_filtered_table);  // TABLE: convert pcl::PointXYZ to pcl::PCLPointCloud2 
      pcl_conversions::fromPCL(*cloud_filtered_table, *output_table);         // TABLE: convert PCLPointCloud2 to sensor_msgs::msg::PointCloud2

      // Object
      auto output = new sensor_msgs::msg::PointCloud2;                        // OBJ: container for sensor_msgs::msg::PointCloud2
      pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);       // OBJ: container for pcl::PCLPointCloud2    
      pcl::toPCLPointCloud2(*XYZcloud_filtered,*cloud_filtered);              // OBJ: convert pcl::PointXYZ to pcl::PCLPointCloud2 
      pcl_conversions::fromPCL(*cloud_filtered, *output);                     // OBJ: convert PCLPointCloud2 to sensor_msgs::msg::PointCloud2

      segmented_pub_->publish(*output);                                        // publish OBJECT plane to /objectPoints
      table_pub_->publish(*output_table);                                      // publish TABLE plane to /tablePoints
      
    };
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}
