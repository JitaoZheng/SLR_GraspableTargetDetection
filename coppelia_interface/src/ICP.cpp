#include <iostream>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>

using namespace std::chrono_literals;

class ICPPointCloudProcessor : public rclcpp::Node
{
public:
    ICPPointCloudProcessor() : Node("icp_pointcloud_processor")
    {
        // Initialize subscriber to the necessary topics
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/converted_pcd_1", 10, std::bind(&ICPPointCloudProcessor::pointcloud_callback, this, std::placeholders::_1));
        
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, std::bind(&ICPPointCloudProcessor::tf_callback, this, std::placeholders::_1));

        // Initialize publisher for the updated point cloud
        icp_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/icp_pcd", 10);

        // Initialize the accumulated point cloud
        accumulated_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        // Convert the incoming ROS PointCloud2 message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *current_cloud);

        if (!accumulated_cloud_->empty())
        {
            // Apply the ICP algorithm to align the current point cloud with the accumulated point cloud
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(current_cloud);
            icp.setInputTarget(accumulated_cloud_);
            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);

            if (icp.hasConverged())
            {
                RCLCPP_INFO(this->get_logger(), "ICP has converged with score: %f", icp.getFitnessScore());
                *accumulated_cloud_ += Final;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
            }
        }
        else
        {
            *accumulated_cloud_ = *current_cloud;
        }

        // Publish the updated point cloud
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*accumulated_cloud_, output);
        output.header.frame_id = "world";
        output.header.stamp = this->get_clock()->now();
        icp_pub_->publish(output);
    }

    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr tf_msg)
    {
        for (const auto &transform : tf_msg->transforms)
        {
            if (transform.header.frame_id == "world" && transform.child_frame_id == "camera_depth_optical_frame")
            {
                // Convert the geometry_msgs::msg::Transform to an Eigen transformation
                Eigen::Affine3d tf_eigen = tf2::transformToEigen(transform.transform);

                // Apply the transformation to the accumulated point cloud
                pcl::transformPointCloud(*accumulated_cloud_, *accumulated_cloud_, tf_eigen);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr icp_pub_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;  // The accumulated point cloud
};

int main(int argc, char **argv)
{
    // Initialize the ROS2 node
    rclcpp::init(argc, argv);

    // Create the node and spin it
    rclcpp::spin(std::make_shared<ICPPointCloudProcessor>());

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
