#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/common/common.h>
//#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{

public:
    MinimalSubscriber(): Node("pc_subscriber"), viewer("PCL Viewer")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "realsense/points", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        segmented_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("segmentedPoints", 10);
        
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
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      RCLCPP_INFO_STREAM(get_logger(), "[Input PointCloud] width " << msg->width << " height " << msg->height);

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

      std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                    << coefficients->values[1] << " "
                                    << coefficients->values[2] << " " 
                                    << coefficients->values[3] << std::endl;
      std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

      // Extract the inliers
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloud_filtered(new pcl::PointCloud<pcl::PointXYZ>); // container for pcl::PointXYZ
      extract.setInputCloud (XYZcloudPtr);
      extract.setIndices (inliers);
      extract.setNegative (false);  // false -> major plane
      extract.filter (*XYZcloud_filtered);

      std::cout << "Passed RANSAC" << std::endl; 
      // // Distance Thresholding: Filter out points that are too far away, e.g. the floor
      // auto plength = XYZcloud_filtered->size();   // Size of the point cloud
      // pcl::PointIndices::Ptr farpoints(new pcl::PointIndices());  // Container for the indices
      // for (int p = 0; p < plength; p++)
      // {
      // // Calculate the distance from the origin/camera
      // float distance = (XYZcloud_filtered->points[p].x * XYZcloud_filtered->points[p].x) +
      //                  (XYZcloud_filtered->points[p].y * XYZcloud_filtered->points[p].y) + 
      //                  (XYZcloud_filtered->points[p].z * XYZcloud_filtered->points[p].z);
      
      //   if (distance > 1.5) // Threshold = 1.5
      //   {
      //     farpoints->indices.push_back(p);    // Store the points that should be filtered out
      //   }
      // }

      // // Extract the filtered point cloud
      // extract.setInputCloud(XYZcloud_filtered);
      // extract.setIndices(farpoints);          // Filter out the far points
      // extract.setNegative(true);
      // extract.filter(*XYZcloud_filtered);
      
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (XYZcloud_filtered);

      std::cout << "Passed setting input cloud" << std::endl; 
      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);
      std::cout << "Passed Kd tree" << std::endl; 
      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.05);
      std::cout << "Passed radius search" << std::endl; 
      // Compute the features
      ne.compute (*cloud_normals); // output <pc
      std::cout << "Passed computation" << std::endl; 


      // visualize normals
      viewer.removePointCloud();
      viewer.setBackgroundColor (0.0, 0.0, 0.5);
      viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(XYZcloud_filtered, cloud_normals);
      viewer.showCloud(XYZcloud_filtered);
      viewer.showCloud(cloud_normals);
      // while (!viewer.wasStopped ())
      // {
      //   viewer.spinOnce ();
      // }

      
      // Convert to ROS data type (sensor_msgs::msg::PointCloud2) for Rviz Visualizer
      // pcl::PointXYZ -> pcl::PCLPointCloud2 -> sensor_msgs::msg::PointCloud2
      auto output = new sensor_msgs::msg::PointCloud2;                  // container for sensor_msgs::msg::PointCloud2
      pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2); // container for pcl::PCLPointCloud2
      pcl::toPCLPointCloud2(*XYZcloud_filtered,*cloud_filtered);        // convert pcl::PointXYZ to pcl::PCLPointCloud2 
      pcl_conversions::fromPCL(*cloud_filtered, *output);               // convert PCLPointCloud2 to sensor_msgs::msg::PointCloud2
      segmented_pub->publish(*output);                                  // publish major plane to /segmentedPoints
        
    };
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}