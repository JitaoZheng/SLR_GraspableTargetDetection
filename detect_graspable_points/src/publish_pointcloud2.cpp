
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

using namespace std;
using namespace std::chrono_literals;

class PointCloudPublisher : public rclcpp::Node
{
public:
    PointCloudPublisher() : Node("pointcloud_publisher")
    {
        // Create publishers for the point clouds
        pub1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("graspability_map", 10);
        pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("peaks_map", 10);

        // Timer to trigger the callback periodically (1 Hz)
        timer_ = this->create_wall_timer(1s, std::bind(&PointCloudPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Load point clouds from PCD files
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> peaks;

        if (pcl::io::loadPCDFile<pcl::PointXYZ>("../ros2_ws/src/detect_graspable_points/pcd/graspability_map_artificial_rocks_pre-scanned.pcd", cloud) == -1) 
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read merged point cloud file");
            return;
        }

        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("../ros2_ws/src/detect_graspable_points/pcd/peaks_linear_0.pcd", peaks) == -1) 
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read peaks point cloud file");
            return;
        }

        // Convert the point clouds to ROS messages
        sensor_msgs::msg::PointCloud2 output1;
        sensor_msgs::msg::PointCloud2 output2;

        //pcl::toROSMsg(cloud, output1);
        pcl::toROSMsg(peaks, output2);

        // Set the header information
        output1.header.frame_id = "camera_depth_optical_frame";
        output1.header.stamp = this->get_clock()->now();

        output2.header.frame_id = "camera_depth_optical_frame";
        output2.header.stamp = this->get_clock()->now();

        // Publish the point clouds
        pub1_->publish(output1);
        pub2_->publish(output2);

        RCLCPP_INFO(this->get_logger(), "PointCloud published under the frame 'map'");
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub1_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub2_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the node and spin it
    rclcpp::spin(std::make_shared<PointCloudPublisher>());

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
