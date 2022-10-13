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

#include <Eigen/Core>
#include <math.h>
#include <array.h>
#include <vector.h>

using std::placeholders::_1;

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
    

// void cloud_cb(const boost::shared_ptr<const sensor_msgs::msg::PointCloud2>& input){

//     //pcl::PCLPointCloud2 pcl_pc2;
//     //pcl_conversions::toPCL(*input,pcl_pc2);
//     //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     //pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
//     //do stuff with temp_cloud here
//     }


private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmented_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr table_pub_;


  std::vector<std::array<Eigen::vector4f>>  grasp_metric(Eigen::Matrix4f normal, Eigen::Matrix4f contact_points, Eigen::Vector4f centroid){
    // inputs: normals directed towards the contact point, centroid of the point cloud
    // outputs: grasp quality metric
      
    // check constraint
    double stable_grasp_angle = 0;

    //define angle threshold
    double pi = 3.14159265359;
    angle_threshold_degree = 5
    angle_threshold = angle_threshold_degree * (pi / 180);

    for(size_t i=0; i<normals.rows(); i++){
      Eigen::Vector3f C1O = (contact_points[i]-centroid).head<3>();  //Get vector between centroid and contact point 1
      Eigen::Vector3f C1N = normal[i].head<3>();   //Vector4f normal for contaCT POINT c1
      for(size_t j=0; j<normals.rows(); j++){
        if(i==j)
          continue;

        Eigen::Vector3f C2O = (contact_points[j]-centroid).head<3>();
        Eigen::Vector3f C2N = (normal[j]).head<3>();

        Eigen::Vector3f collinear = C1O.cross(C2O);

        std::vector<std::array<Eigen::vector4f>> cp_pairs;

        if(collinear.isZero(1)){   // FIX
          //if collinear do following
          C1C2 = (contact_points[i]-contact_points[j]).head<3>();
          magnitude_C1C2 = (C1C2.square()).sum().sqrt();
          magnitude_C1N = (C1N.square()).sum().sqrt();
          magnitude_C2N = (C2N.square()).sum().sqrt();
          
          angle1 = acos(C1C2.dot(C1N)/(magnitude_C1C2*magnitude_C1N));
          angle2 = acos(C1C2.dot(C2N)/(magnitude_C1C2*magnitude_C2N));

          grasp_angle = angle1+angle2;

          stable_grasp_angle_new = max(stable_grasp_angle, 180 - grasp_angle);
          if(stable_grasp_angle_new>pi-angle_threshold || stable_grasp_angle_new<pi+angle_threshold){
            cp_pairs.push_back([contact_points[i], contact_points[j]])
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
      RCLCPP_INFO_STREAM(get_logger(), "# of point cloud after downsampling: " << XYZcloudPtr->size () << " points" );
      
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
      RCLCPP_INFO_STREAM(this->get_logger(), "# of object point cloud: " << XYZcloud_filtered->size ());


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
      ne.setRadiusSearch (0.05);
      ne.useSensorOriginAsViewPoint();

      // Compute the features
      ne.compute (*cloud_normals);
      RCLCPP_INFO_STREAM(this->get_logger(), "# of normals: " << cloud_normals->size ());

      // CENTROID
      // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
      Eigen::Vector4f xyz_centroid;
      
      // Estimate the XYZ centroid
      pcl::compute3DCentroid (*XYZcloud_filtered, xyz_centroid);

      // Fill in the cloud data
      pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      
      RCLCPP_INFO_STREAM(this->get_logger(), "xyz_centroid: " << xyz_centroid[0]);
      RCLCPP_INFO_STREAM(this->get_logger(), ", " << xyz_centroid[1]);
      RCLCPP_INFO_STREAM(this->get_logger(), ", " << xyz_centroid[2]);
    

      pcl::PointXYZ centroidXYZ(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);

      // FLIPPING NORMALS ACCORIDNG TO CENTROID
      Eigen::Matrix3Xf normal_vector_matrix(3,cloud_normals->size());
      for(size_t i = 0; i < cloud_normals->size(); i++) {
        Eigen::Vector4f normal = cloud_normals->at(i).getNormalVector4fMap();
        Eigen::Vector4f normal_dup = cloud_normals->at(i).getNormalVector4fMap();

        //pcl::flipNormalTowardsViewpoint(centroidXYZ, 0, 0, 0, normal);
        pcl::flipNormalTowardsViewpoint(XYZcloud_filtered->at(i), xyz_centroid[0], xyz_centroid[1], xyz_centroid[2], normal);
        normal_vector_matrix(0,i) = normal[0];
        normal_vector_matrix(1,i) = normal[1];
        normal_vector_matrix(2,i) = normal[2];
      }
        

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
      RCLCPP_INFO_STREAM(this->get_logger(), "Normal Vector Matrix dims:(" << normal_vector_matrix.rows() << "," << normal_vector_matrix.cols() << ")");

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