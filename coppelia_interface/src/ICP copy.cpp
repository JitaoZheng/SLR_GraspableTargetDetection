#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
//#include <geometry_msgs/msg/Quaternion.hpp>
//#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <pcl/filters/filter.h> // for NaN removal

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>  
//#include <tf/transform_broadcaster.h> 

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;
using namespace std;

class PointCloudRegistration : public rclcpp::Node
{
public:
    PointCloudRegistration() 
    : Node("point_cloud_registration")
    //: viewer("ICP Viewer") 
    {
        rclcpp::QoS qos(10);

        cumulative_transform.setIdentity();
        initial_position_set = false; //  used to ensure that the initial position is only set once.
        

        subscriptionPCD = this->create_subscription<sensor_msgs::msg::PointCloud2>("/merged_pcd", 10, std::bind(&PointCloudRegistration::cloudCallback, this, std::placeholders::_1));
        subscriptionPos = this->create_subscription<geometry_msgs::msg::Vector3>("/body_pos_copp", 10, std::bind(&PointCloudRegistration::posCallback, this, std::placeholders::_1));
        subscriptionOri = this->create_subscription<geometry_msgs::msg::Quaternion>("/body_quat_copp", 10, std::bind(&PointCloudRegistration::oriCallback, this, std::placeholders::_1));

        
        merged_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/icp_pcd", qos);
        
       
    }

private:

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        PCDbuffer = *msg;
        PointCloud input_cloud_raw;
        PointCloud input_cloud;
        pcl::fromROSMsg(PCDbuffer, input_cloud_raw);

        // remove NaNs
        vector<int> indices;
        pcl::removeNaNFromPointCloud(input_cloud_raw, input_cloud, indices);

        // If it's the first cloud, just store it and set the initial position
        if (!prev_cloud_loaded)
        {
            prev_cloud = input_cloud;
            prev_cloud_loaded = true;

            initial_position_set = true;

            // Set initial position based on the received messages
            //setInitialPosition();
            return;
        }

        // Get the current position and orientation from the received messages
        Eigen::Matrix4f current_transform = getCurrentTransform();

        // Perform ICP
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setInputSource(input_cloud.makeShared());
        icp.setInputTarget(prev_cloud.makeShared());

        PointCloud transformed_cloud;
        icp.align(transformed_cloud, current_transform);

        // Update the cumulative transformation
        //cumulative_transform *= icp.getFinalTransformation();

        // Concatenate the transformed cloud with the previous cloud
        pcl::transformPointCloud(input_cloud, transformed_cloud, icp.getFinalTransformation());
        merged_cloud += transformed_cloud;

        

        // Publish the merged cloud
        sensor_msgs::msg::PointCloud2 merged_msg;
        pcl::toROSMsg(merged_cloud, merged_msg);
        merged_msg.header = PCDbuffer.header;

        merged_cloud_pub->publish(merged_msg);

        // Publish the cumulative transformation as a transform
        //publishTransform(cumulative_transform, PCDbuffer.header.stamp);

        // Update the previous cloud for the next iteration
        prev_cloud = merged_cloud;


        
        pcl::io::savePCDFileASCII("/home/jitao/catkin_ws/src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/merged_pcd_icp.pcd", merged_cloud);
        RCLCPP_INFO(this->get_logger(), "ICP-Merged point cloud created and published.");

    }

    void posCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr position_msg)
    {
        current_position = *position_msg;
    }

    void oriCallback(const geometry_msgs::msg::Quaternion::ConstSharedPtr orientation_msg)
    {
        current_orientation = *orientation_msg;
    }
/* 
    void setInitialPosition()
    {
        // Set initial position based on the received messages
        initial_transform.setIdentity();

        if (!initial_position_set)
        {
            initial_transform(0, 3) = current_position.x;
            initial_transform(1, 3) = current_position.y;
            initial_transform(2, 3) = current_position.z;
        }

        initial_position_set = true;
    }
 */
    Eigen::Matrix4f getCurrentTransform()
    {
        Eigen::Matrix4f current_transform = Eigen::Matrix4f::Identity();

        // Get the current position and orientation from the received messages
        if (initial_position_set)
        {
            current_transform(0, 3) = current_position.x;
            current_transform(1, 3) = current_position.y;
            current_transform(2, 3) = current_position.z;

            Eigen::Quaternionf quaternion(current_orientation.w, current_orientation.x, current_orientation.y, current_orientation.z);
            Eigen::Matrix3f rotation_matrix = quaternion.toRotationMatrix();
            current_transform.block<3, 3>(0, 0) = rotation_matrix;
        }

        return current_transform;
    }

/* 

    void publishTransform(const Eigen::Matrix4f &transform, const rclcpp::Time &timestamp)
    {
        tf::Transform tf_transform;
        tf_transform.setOrigin(tf::Vector3(transform(0, 3), transform(1, 3), transform(2, 3)));

        Eigen::Matrix3f rotation_matrix = transform.block<3, 3>(0, 0);
        Eigen::Quaternionf quaternion(rotation_matrix);
        tf_transform.setRotation(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));

        tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, timestamp, "initial_frame", "current_frame"));
    }
 */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriptionPCD;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriptionPos;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr subscriptionOri;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub;
    
    sensor_msgs::msg::PointCloud2 PCDbuffer;
    PointCloud prev_cloud;
    PointCloud merged_cloud;
    //tf2::TransformBroadcaster tf_broadcaster; // to be checked

    bool prev_cloud_loaded = false;
    Eigen::Matrix4f cumulative_transform;
    bool initial_position_set;
    //Eigen::Matrix4f initial_transform;
    //rclcpp::Time initial_position_time;
    //geometry_msgs::msg::Vector3 pos_data;
    //geometry_msgs::msg::Quaternion ori_data;

    //geometry_msgs::msg::Vector3 initial_position;
    geometry_msgs::msg::Vector3 current_position;
    geometry_msgs::msg::Quaternion current_orientation;
    
};

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PointCloudRegistration>());

    rclcpp::shutdown();
    return 0;
}
