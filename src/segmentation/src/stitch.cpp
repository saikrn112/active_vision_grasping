#include <iostream>
#include <optional>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"

class Stitcher : public rclcpp::Node
{

public:
    Stitcher()
        : Node("stitcher")
        , toFrameRel("world")
        , fromFrameRel("camera_link")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( "/objectPoints", 
                                                                        10, 
                                                                        std::bind(&Stitcher::topic_callback, 
                                                                        this, std::placeholders::_1));

        stitched_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/stitchedObject", 10);

        // tf2 related
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stitched_pub_;
    pcl::PointCloud<pcl::PointXYZ> old_stitched_point_cloud;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::optional<Eigen::Matrix4d> reference_transform;

    std::string toFrameRel;
    std::string fromFrameRel;

    pcl::PointCloud<pcl::PointXYZ>::Ptr 
        sensorMsgToXYZCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        // Convert sensor_msg::PointCloud2 to pcl::PCLPointCloud2
        pcl::PCLPointCloud2::Ptr msg_cloudPtr(new pcl::PCLPointCloud2);

        pcl_conversions::toPCL(*msg, *msg_cloudPtr);

        // Convert pcl::PCLPointCloud2 to PointXYZ data type
        pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*msg_cloudPtr,*XYZcloudPtr);
        return XYZcloudPtr;
    }

    std::shared_ptr<sensor_msgs::msg::PointCloud2>
        XYZCloudToSensorMsg(const pcl::PointCloud<pcl::PointXYZ>& cloud) const
    {

        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        pcl::PCLPointCloud2::Ptr pcl2_cloud(new pcl::PCLPointCloud2);

        pcl::toPCLPointCloud2(cloud,*pcl2_cloud);

        pcl_conversions::fromPCL(*pcl2_cloud, *cloud_msg);

        cloud_msg->header.frame_id = "world";

        return cloud_msg;
    }

    std::optional<Eigen::Matrix4d> get_transform() const
    {
        geometry_msgs::msg::TransformStamped t;

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        try 
        {
            t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        } 
        catch (const tf2::TransformException & ex) 
        {
            RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return {};
        }

        auto eigen_transform = tf2::transformToEigen(t);
        Eigen::Matrix4d data = eigen_transform.matrix();
        return {data};
    }


    template<typename T>
    bool is_tolerant(const T& val, double threshold, double eps = 0.1) const
    {
        if (threshold - eps <= val && val <= threshold + eps)
        {
            return true;
        }

        return false;
    }

    struct TStruct
    {
        Eigen::Vector3d trans;
        Eigen::Vector3d axis;
        double angle;
    };

    TStruct convertTtoAA(const Eigen::Matrix4d& T) const
    {

        Eigen::Matrix3d rot = T(Eigen::seqN(0,3),Eigen::seqN(0,3));
        Eigen::Vector3d trans = T(Eigen::seqN(0,3),3);

        auto axisAngle = Eigen::AngleAxisd(rot);
        const auto& axis = axisAngle.axis().normalized();
        const auto& angle = axisAngle.angle();
        
        return {trans, axis, angle};

    }

    bool is_transformation_changed( const Eigen::Matrix4d& newT, 
                                    double eps = 0.1) const
    {

        if (!reference_transform)
        {
            return false;
        }

        auto [newTrans, newAxis, newAngle] = convertTtoAA(newT);
        auto [oldTrans, oldAxis, oldAngle] = convertTtoAA(*reference_transform);


        auto axis_range = newAxis.dot(oldAxis);
        auto transDist = (newTrans - oldTrans).squaredNorm();
        if (is_tolerant(axis_range, 0, eps)
            && is_tolerant(newAngle, oldAngle, eps)
            && is_tolerant(transDist,0, eps))
        {
            return false;
        }

        return true;
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

        // 1. get transformation - rotation and translation
        auto latest_transform_optional = get_transform();
        if (!latest_transform_optional)
        {
            return;
        }
        auto latest_transform = *latest_transform_optional;
        RCLCPP_INFO_STREAM(get_logger(), "got transform: " << latest_transform);

        // 2. check if transform is initialized
        if (!reference_transform)
        {
            reference_transform = latest_transform;
            stitched_pub_->publish(*msg);
            old_stitched_point_cloud = *sensorMsgToXYZCloud(msg);
            return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "transform is initialized");

        // 3. check if transform has changed
        if (!is_transformation_changed(latest_transform))
        {
            stitched_pub_->publish(*msg);
            old_stitched_point_cloud = *sensorMsgToXYZCloud(msg);
            return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "transform has changed, SERIOUS MODE -_-");

        // 4. transform the callback point cloud using the above
        auto input_cloud = sensorMsgToXYZCloud(msg);
        decltype(input_cloud) transformed_cloud;
        pcl::transformPointCloud(*input_cloud,*transformed_cloud, latest_transform);
        RCLCPP_INFO_STREAM(get_logger(), "transformed point cloud");

        // 5. concatenate with the old stitched point cloud
        old_stitched_point_cloud += *transformed_cloud;
        RCLCPP_INFO_STREAM(get_logger(), "stitched cloud, it's gonna rain");

        // 6. publish
        auto out_msg = XYZCloudToSensorMsg(old_stitched_point_cloud);
        stitched_pub_->publish(*out_msg);
        return;

    }

    // 1. restart the world
    // 2. segment 
    // 3. check the object is segmented in rviz
    // 4. run the stitcher
    // 5. should publish the cloud as it is
    // 6. get contact pairs
    // 7. change the position using the python client
    // 8. 

};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stitcher>());
  rclcpp::shutdown();
  return 0;
}

