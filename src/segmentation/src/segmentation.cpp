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

# include <Eigen/Core>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{

public:
    MinimalSubscriber(): Node("pc_subscriber"), viewer("PCL Viewer")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "realsense/points", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        segmented_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("objectPoints", 10);
        table_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("tablePoints", 10);
        centroid_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("centroidPoint", 10);
        
    }

// void cloud_cb(const boost::shared_ptr<const sensor_msgs::msg::PointCloud2>& input){

//     //pcl::PCLPointCloud2 pcl_pc2;
//     //pcl_conversions::toPCL(*input,pcl_pc2);
//     //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     //pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
//     //do stuff with temp_cloud here
//     }

private:
    pcl::visualization::PCLVisualizer viewer;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmented_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr table_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr centroid_pub;
    
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      //RCLCPP_INFO_STREAM(get_logger(), "[Input PointCloud] width " << msg->width << " height " << msg->height);

      // Convert sensor_msg::PointCloud2 to pcl::PCLPointCloud2
      pcl::PCLPointCloud2::Ptr cloudPtr(new pcl::PCLPointCloud2); // container for pcl::PCLPointCloud
      pcl_conversions::toPCL(*msg, *cloudPtr); // convert to PCLPointCloud2 data type

      // Downsample 
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloudPtr);
      sor.setLeafSize (0.01f, 0.01f, 0.01f);
      sor.filter (*cloudPtr);

      // Convert pcl::PCLPointCloud2 to PointXYZ data type
      pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr(new pcl::PointCloud<pcl::PointXYZ>); // container for pcl::PointXYZ
      pcl::fromPCLPointCloud2(*cloudPtr,*XYZcloudPtr);  // convert to pcl::PointXYZ data type

      // Printing point cloud data
      std::cerr << "Point cloud data: " << XYZcloudPtr->size () << " points" << std::endl;
      
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
      ne.setRadiusSearch (0.1);

      // Compute the features
      ne.compute (*cloud_normals);

      std::cout << "Selected a few normals to display" << std::endl;
      for(size_t i = 0; i < cloud_normals->size(); i+=95) {
        std::cout << XYZcloud_filtered->at(i) << std::endl;
        std::cout << cloud_normals->at(i) << std::endl;
        std::cout << std::endl;
      }

      // CENTROID
      // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
      Eigen::Vector4f xyz_centroid;
      // Estimate the XYZ centroid
      pcl::compute3DCentroid (*XYZcloud_filtered, xyz_centroid);

      // Fill in the cloud data
      pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      
      std::cerr << "xyz_centroid: " << xyz_centroid[0];
      std::cerr << ", " << xyz_centroid[1];
      std::cerr << ", " << xyz_centroid[2] << std::endl;
    

      std::cout << "flipped" << std::endl;
    
      // FLIPPING NORMALS ACCORIDNG TO CENTROID
      for(size_t i = 0; i < cloud_normals->size(); i+=95) {
        // !! TODO: Making this work
        // pcl::flipNormalTowardsViewpoint(xyz_centroid, 0, 0, 0,
				//       cloud_normals->at(i).normal[0],
				//       cloud_normals->at(i).normal[1],
				//       cloud_normals->at(i).normal[2]);
        std::cout << XYZcloud_filtered->at(i) << std::endl;
        std::cout << cloud_normals->at(i) << std::endl;
        std::cout << std::endl;
      }

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
      segmented_pub->publish(*output);                                        // publish OBJECT plane to /objectPoints
      table_pub->publish(*output_table);                                      // publish TABLE plane to /tablePoints
      centroid_pub->publish(*output_centroid);
    };
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}