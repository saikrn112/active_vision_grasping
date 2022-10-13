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

using std::placeholders::_1;
using Eigen::placeholders::all;

class PointCloudProcessor : public rclcpp::Node
{

public:
    PointCloudProcessor(): Node("pc_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "realsense/points", 10, std::bind(&PointCloudProcessor::topic_callback, this, _1));
      segmented_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("objectPoints", 10);
      table_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tablePoints", 10);       
    }


private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmented_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr table_pub_;

  bool isCollinear(const Eigen::Vector3f& vec1, const Eigen::Vector3f& vec2, double eps = 0.1) const
  {
    const auto& vec1_norm = vec1.normalized();
    const auto& vec2_norm = vec2.normalized();
    auto dot_product_val = vec1_norm.dot(vec2_norm);
    // RCLCPP_INFO_STREAM(get_logger(), "dot_product_val: " << dot_product_val 
    //               << " vec1:" << vec1
    //               << " vec2:" << vec2);

    if ( (-1 - eps <= dot_product_val) && (dot_product_val <= -1 + eps))
    {
      return true;
    } 
    return false;
  }

  auto grasp_metric(const Eigen::Matrix3Xf& normals, const Eigen::Matrix3Xf& contact_points, const Eigen::Vector3f& centroid) const
  {
    // inputs: normals directed towards the contact point, centroid of the point cloud
    // outputs: grasp quality metric
      
    // check constraint
    double stable_grasp_angle = 0;

    //define angle threshold

    float angle_threshold_degree = 10;
    float angle_threshold = angle_threshold_degree * (M_PI / 180);

    std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f>> cp_pairs;

    auto CX0 = contact_points.colwise() - centroid;

    
    for (size_t i=0; i < normals.cols(); i++)
    {
      // Eigen::Vector3f C1O = contact_points[i] - centroid;  //Get vector between centroid and contact point 1
      const auto& C1N = normals(all, i);   //Vector4f normal for contaCT POINT c1
      
      const auto& C1 = contact_points(all, i);
      const auto& C10 = CX0(all, i);

      for (size_t j=0; j<normals.cols(); j++)
      {  
        if (i==j)
        {
          continue;
        }

        const auto& C2 = contact_points(all, j);
        const auto& C20 = CX0(all, j);

        auto is_collinear = isCollinear(C10,C20);
        // RCLCPP_INFO_STREAM(get_logger(), "Collinear of" << is_collinear );

        if (is_collinear)
        {
          const auto& C2N = normals(all, j);
          // vector between contact points
          auto C1C2 = (C1-C2).normalized();
          // calculate angles between contact points and 
          auto angle1 = acos(C1N.dot(C1C2));
          auto angle2 = acos(C2N.dot(C1C2));

          double grasp_angle = angle1 + angle2;

          stable_grasp_angle = std::max(stable_grasp_angle, grasp_angle);
          
          // RCLCPP_INFO_STREAM(get_logger(), "stable_grasp_angle" << stable_grasp_angle );
          if(M_PI - angle_threshold < stable_grasp_angle && stable_grasp_angle < M_PI + angle_threshold)
          {
            cp_pairs.push_back({C1, C2});
            // RCLCPP_INFO_STREAM(get_logger(), "stable_grasp_angle within threshold " << stable_grasp_angle );
          }
        }
      }
    }
    return cp_pairs;
  }    
    
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      // Convert sensor_msg::PointCloud2 to pcl::PCLPointCloud2
      pcl::PCLPointCloud2::Ptr cloudPtr(new pcl::PCLPointCloud2); // container for pcl::PCLPointCloud
      pcl_conversions::toPCL(*msg, *cloudPtr); // convert to PCLPointCloud2 data type

      // Downsample 
      // auto downsampledCloudPtr = downsample(cloudPtr);
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloudPtr);
      sor.setLeafSize (0.005f, 0.005f, 0.005f);
      sor.filter (*cloudPtr);

      // Convert pcl::PCLPointCloud2 to PointXYZ data type
      pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr(new pcl::PointCloud<pcl::PointXYZ>); // container for pcl::PointXYZ
      pcl::fromPCLPointCloud2(*cloudPtr,*XYZcloudPtr);  // convert to pcl::PointXYZ data type

      // Printing point cloud data
      // RCLCPP_INFO_STREAM(get_logger(), "# of point cloud after downsampling: " << XYZcloudPtr->size () << " points" );
      
      // Distance Thresholding: Filter out points that are too far away, e.g. the floor
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

      // Extract the filtered point cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(XYZcloudPtr);
      extract.setIndices(farpoints);          // Filter out the far points
      extract.setNegative(true);
      extract.filter(*XYZcloudPtr);

      // RANSAC; Plane model segmentation from pcl
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);

      seg.setInputCloud (XYZcloudPtr);
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size () == 0)
      {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
      }

      // Extract the inliers
      //pcl::ExtractIndices<pcl::PointXYZ> extract;
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
      // RCLCPP_INFO_STREAM(this->get_logger(), "# of object point cloud: " << XYZcloud_filtered->size ());


      // NORMAL ESTIMATION
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (XYZcloud_filtered);

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);

      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.005);
      ne.useSensorOriginAsViewPoint();

      // Compute the features
      ne.compute (*cloud_normals);
      // RCLCPP_INFO_STREAM(this->get_logger(), "# of normals: " << cloud_normals->size ());

      // CENTROID
      // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
      Eigen::Vector4f xyz_centroid;
      
      // Estimate the XYZ centroid
      pcl::compute3DCentroid (*XYZcloud_filtered, xyz_centroid);

      // Fill in the cloud data
      pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      
      // RCLCPP_INFO_STREAM(this->get_logger(), "xyz_centroid: " << xyz_centroid[0]);
      RCLCPP_INFO_STREAM(this->get_logger(), ", " << xyz_centroid[1]);
      RCLCPP_INFO_STREAM(this->get_logger(), ", " << xyz_centroid[2]);
    

      pcl::PointXYZ centroidXYZ(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);

      // FLIPPING NORMALS ACCORIDNG TO CENTROID
      Eigen::Matrix3Xf normal_vector_matrix(3,cloud_normals->size());
      Eigen::Matrix3Xf point_cloud(3,cloud_normals->size());
      for(size_t i = 0; i < cloud_normals->size(); i++) {
        Eigen::Vector3f normal = cloud_normals->at(i).getNormalVector4fMap().head(3);
        Eigen::Vector3f normal_dup = cloud_normals->at(i).getNormalVector4fMap().head(3);

        //pcl::flipNormalTowardsViewpoint(centroidXYZ, 0, 0, 0, normal);
        pcl::flipNormalTowardsViewpoint(XYZcloud_filtered->at(i), xyz_centroid[0], xyz_centroid[1], xyz_centroid[2], normal);
        normal_vector_matrix(0,i) = normal[0];
        normal_vector_matrix(1,i) = normal[1];
        normal_vector_matrix(2,i) = normal[2];

        //const auto& pointMatrix = XYZcloud_filtered->at(i);
        point_cloud(0,i) = XYZcloud_filtered->points[i].x;
        point_cloud(1,i) = XYZcloud_filtered->points[i].y;
        point_cloud(2,i) = XYZcloud_filtered->points[i].z;
      }

      

      const auto& data = grasp_metric(normal_vector_matrix, point_cloud,xyz_centroid.head(3));
      RCLCPP_INFO_STREAM(this->get_logger(), "Size of data: " << data.size());
      
      
      //   float eps = 0.5;
      //   float dot_range = normal_dup.dot(normal);
      //   std::cout << "before flip:" << dot_range << std::endl;
      //   if (dot_range <= -1 + eps && dot_range >= -1 -eps){
      //     std::cout << "dot range: " << dot_range<< std::endl;
      //     std::cout << "pointcloud: " << XYZcloud_filtered->at(i) << std::endl;
      //     std::cout << "normal: " << normal_dup << std::endl;
      //     std::cout << "normal flipped: " << normal << std::endl;
      //     std::cout << std::endl;  
      //   } 
      // RCLCPP_INFO_STREAM(this->get_logger(), "Normal Vector Matrix dims:(" << normal_vector_matrix.rows() << "," << normal_vector_matrix.cols() << ")");

      // Convert to ROS data type (sensor_msgs::msg::PointCloud2) for Rviz Visualizer
      // pcl::PointXYZ -> pcl::PCLPointCloud2 -> sensor_msgs::msg::PointCloud2

      // Centroid
      XYZcloud_filtered_table->push_back(pcl::PointXYZ(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]));
      auto output_centroid = new sensor_msgs::msg::PointCloud2;                  // TABLE: container for sensor_msgs::msg::PointCloud2
      pcl::PCLPointCloud2::Ptr cloud2_centroid(new pcl::PCLPointCloud2); // TABLE: container for pcl::PCLPointCloud2
      pcl::toPCLPointCloud2(*centroid_cloud,*cloud2_centroid);  // TABLE: convert pcl::PointXYZ to pcl::PCLPointCloud2 
      pcl_conversions::fromPCL(*cloud2_centroid, *output_centroid);         // TABLE: convert PCLPointCloud2 to sensor_msgs::msg::PointCloud2

      // Table
      auto output_table = new sensor_msgs::msg::PointCloud2;                  // TABLE: container for sensor_msgs::msg::PointCloud2
      pcl::PCLPointCloud2::Ptr cloud_filtered_table(new pcl::PCLPointCloud2); // TABLE: container for pcl::PCLPointCloud2
      pcl::toPCLPointCloud2(*XYZcloud_filtered_table,*cloud_filtered_table);  // TABLE: convert pcl::PointXYZ to pcl::PCLPointCloud2 
      pcl_conversions::fromPCL(*cloud_filtered_table, *output_table);         // TABLE: convert PCLPointCloud2 to sensor_msgs::msg::PointCloud2

      // Object
      auto output = new sensor_msgs::msg::PointCloud2;                        // OBJ: container for sensor_msgs::msg::PointCloud2
      pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);       // OBJ: container for pcl::PCLPointCloud2    
      pcl::toPCLPointCloud2(*XYZcloud_filtered,*cloud_filtered);              // OBJ: convert pcl::PointXYZ to pcl::PCLPointCloud2 
      pcl_conversions::fromPCL(*cloud_filtered, *output);                     // OBJ: convert PCLPointCloud2 to sensor_msgs::msg::PointCloud2

      //pcl::io::savePCDFileASCII ("test_pcd.pcd", XYZcloud_filtered);
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