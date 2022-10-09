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

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{

public:
    MinimalSubscriber(): Node("pc_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "realsense/points", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        //pub = this->create_publisher<pcl::PointCloud<pcl::PointXYZ> >("pcl/points", 10);
        //pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("pcdcloud", 1);
    }

void cloud_cb(const boost::shared_ptr<const sensor_msgs::msg::PointCloud2>& input){

		//pcl::PCLPointCloud2 pcl_pc2;
		//pcl_conversions::toPCL(*input,pcl_pc2);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
		//do stuff with temp_cloud here
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    //rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZ>>::SharedPtr pub;
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      RCLCPP_INFO_STREAM(get_logger(), "[Input PointCloud] width " << msg->width << " height " << msg->height);

      // Convert sensor_msg::PointCloud2 to pcl::PCLPointCloud2
      
      // Container for original & filtered data
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);    

      // Convert to PCL data type
      pcl_conversions::toPCL(*msg, *cloud);
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      //pcl::fromPCLPointCloud2(cloud,*pt_cloud)

      RCLCPP_INFO_STREAM(get_logger(), "[Output PointCloud] width " << cloud->width << " height " << cloud->height);

      // Convert to ROS data type (ROS publisher not compatible with PCL datatype i think)
      //sensor_msgs::msg::PointCloud2 output = new sensor_msgs::msg::PointCloud2;

        
    };
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}