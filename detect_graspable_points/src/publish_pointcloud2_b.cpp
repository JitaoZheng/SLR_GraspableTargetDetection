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
    PointCloudPublisher() : Node("pointcloud_publisher"), file_index_(0)
    {
        // Create publishers for the point cloud
        pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("graspability_map", 10);

        // Timer to trigger the callback periodically (1 Hz)
        timer_ = this->create_wall_timer(1s, std::bind(&PointCloudPublisher::timer_callback, this));

        // Timer to switch between point clouds every 15 seconds
        switch_timer_ = this->create_wall_timer(15s, std::bind(&PointCloudPublisher::switch_callback, this));
    }

private:
    void timer_callback()
    {
        // Load the current point cloud from the PCD file
        pcl::PointCloud<pcl::PointXYZRGB> cloud;

        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_files_[file_index_], cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read point cloud file %s", pcd_files_[file_index_].c_str());
            return;
        }

        // Convert the point cloud to a ROS message
        sensor_msgs::msg::PointCloud2 output2;
        pcl::toROSMsg(cloud, output2);

        // Set the header information
        output2.header.frame_id = "map";
        output2.header.stamp = this->get_clock()->now();

        // Publish the point cloud
        pub2_->publish(output2);

        RCLCPP_INFO(this->get_logger(), "PointCloud published from file %s under the frame 'map'", pcd_files_[file_index_].c_str());
    }

    void switch_callback()
    {
        // Increment the file index to switch to the next PCD file
        file_index_ = (file_index_ + 1) % pcd_files_.size();
        RCLCPP_INFO(this->get_logger(), "Switching to next point cloud file: %s", pcd_files_[file_index_].c_str());
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub2_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr switch_timer_;

    std::vector<std::string> pcd_files_ = {
        "../ros2_ws/src/detect_graspable_points/pcd/graspability_map_linear_1.pcd",
        "../ros2_ws/src/detect_graspable_points/pcd/graspability_map_linear_2.pcd",
        "../ros2_ws/src/detect_graspable_points/pcd/graspability_map_linear_3.pcd",
        "../ros2_ws/src/detect_graspable_points/pcd/graspability_map_linear_4.pcd",
        "../ros2_ws/src/detect_graspable_points/pcd/graspability_map_linear_5.pcd",
        "../ros2_ws/src/detect_graspable_points/pcd/graspability_map_linear_0.pcd"
    };
    
    size_t file_index_;  // Current index of the PCD file being published
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
//ros2 run tf2_ros static_transform_publisher -1 0 0 -1.5707963 0 0 world map