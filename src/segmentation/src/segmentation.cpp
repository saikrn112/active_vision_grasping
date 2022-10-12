#include <iostream>
#include <memory>
#include <thread>
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

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class PointCloudProcessor : public rclcpp::Node
{

public:
    PointCloudProcessor(pcl::visualization::PCLVisualizer& viewer,
                        int& output_view)
    : Node("pc_subscriber")
    , viewer(viewer)
    , output_view(output_view)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("realsense/points"
                                                                                , 10
                                                                                , std::bind(&PointCloudProcessor::topic_callback, this, _1));

        segmented_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("segmentedPoints", 10);
        
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmented_pub;
    pcl::visualization::PCLVisualizer& viewer;
    int output_view;

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

      viewer.removeAllPointClouds(output_view);
      viewer.addPointCloud<pcl::PointXYZ>(XYZcloudPtr, "output", output_view);


      // Printing point cloud data
      RCLCPP_INFO_STREAM(get_logger(), "Point cloud data: " << XYZcloudPtr->size () << " points");

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
          return;
      }

      std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                    << coefficients->values[1] << " "
                                    << coefficients->values[2] << " " 
                                    << coefficients->values[3] << std::endl;
      std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

      // Extract the inliers
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (XYZcloudPtr);
      extract.setIndices (inliers);
      extract.setNegative (false);  // false -> major plane

      pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloud_filtered(new pcl::PointCloud<pcl::PointXYZ>); // container for pcl::PointXYZ
      extract.filter (*XYZcloud_filtered);

      std::cout << "Passed RANSAC" << std::endl; 
      /*
       // Distance Thresholding: Filter out points that are too far away, e.g. the floor
       auto plength = XYZcloud_filtered->size();   // Size of the point cloud
       pcl::PointIndices::Ptr farpoints(new pcl::PointIndices());  // Container for the indices
       for (int p = 0; p < plength; p++)
       {
       // Calculate the distance from the origin/camera
       float distance = (XYZcloud_filtered->points[p].x * XYZcloud_filtered->points[p].x) +
                        (XYZcloud_filtered->points[p].y * XYZcloud_filtered->points[p].y) + 
                        (XYZcloud_filtered->points[p].z * XYZcloud_filtered->points[p].z);
      
         if (distance > 1.5) // Threshold = 1.5
         {
           farpoints->indices.push_back(p);    // Store the points that should be filtered out
         }
       }

       // Extract the filtered point cloud
       extract.setInputCloud(XYZcloud_filtered);
       extract.setIndices(farpoints);          // Filter out the far points
       extract.setNegative(true);
       extract.filter(*XYZcloud_filtered);
       */
      
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

//      viewer.removeAllPointClouds();
//      viewer.removePointCloud("sphere");
//      pcl::PointXYZ o;
//      o.x = 1.0;
//      o.y = 0;
//      o.z = 0;
//      viewer.addSphere (o, 0.25, "sphere", 0);
//      viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(XYZcloud_filtered, cloud_normals);

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
  rclcpp::Rate loop_rate(10);

    // ------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Generating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  std::uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * std::cos (pcl::deg2rad(angle));
      basic_point.y = sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
              static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = basic_cloud_ptr->size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = point_cloud_ptr->size ();
  point_cloud_ptr->height = 1;

  pcl::visualization::PCLVisualizer viewer ("3D Viewer");

  viewer.getRenderWindow()->GlobalWarningDisplayOff();
  viewer.setBackgroundColor (0, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

//  pcl::visualization::PCLVisualizer viewer("PCL Visualizer");
//  int output_view;
//  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, output_view);
//  viewer.setBackgroundColor(200, 0, 0, output_view);
//
//  viewer.addCoordinateSystem(1.0);
//  viewer.initCameraParameters();
//
//
//  auto pointcloud_processor = std::make_shared<PointCloudProcessor>(viewer,output_view);
//  while (rclcpp::ok() && !viewer.wasStopped())
//  {
//    rclcpp::spin_some(pointcloud_processor);
//    loop_rate.sleep();
//  }
//
//  rclcpp::shutdown();
  return 0;
}
