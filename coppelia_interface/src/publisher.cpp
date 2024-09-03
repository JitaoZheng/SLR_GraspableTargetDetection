#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
    public:
    MinimalPublisher():
    Node("minimal_publisher"), count_(0)
    {
        rclcpp::QoS qos(10);

        publisher_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("info", qos);
        
        timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::TimerCallback, this));
    }
    private:
    void TimerCallback(){
        rclcpp::Time now = this->get_clock()->now();

        float w = 640;
        float h = 360;
        float horizontalAngle = 87;
        float verticalAngle = 58;
        float f_x = (w/2)/tan((horizontalAngle*(M_PI/180))/2);
        float f_y = (h/2)/tan((verticalAngle*(M_PI/180))/2);
        float c_x = w/2;
        float c_y = h/2;

        auto msg_info = sensor_msgs::msg::CameraInfo();
        msg_info.header.stamp = now;
        msg_info.header.frame_id = "a";
        msg_info.height = h;
        msg_info.width = w;
        msg_info.distortion_model = "PLUMB_BOB";
        msg_info.d = {0, 0, 0, 0, 0};
        msg_info.k = {f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1};
        msg_info.r ={1, 0, 0, 0, 1, 0, 0, 0, 1};
        msg_info.p ={f_x, 0, c_x, 0, 0, f_y, c_y, 0, 0, 0, 1, 0};
        msg_info.binning_x = 1;
        msg_info.binning_y = 1;
        msg_info.roi.x_offset = 0;
        msg_info.roi.y_offset = 0;
        msg_info.roi.width = 0;
        msg_info.roi.height = 0;
        msg_info.roi.do_rectify = false;


        publisher_info_->publish(msg_info);
        RCLCPP_INFO(this->get_logger(), "Published camera info.");

        /*  
        auto msg_depth = sensor_msgs::msg::Image();
        msg_depth.header.stamp = now;
        msg_depth.header.frame_id = "a"; 
        msg_depth.height = h;
        msg_depth.width = w;
        msg_depth.encoding = "rgb8";
        msg_depth.is_bigendian = 1;
        msg_depth.step = w*3;
        msg_depth.data = ;
        */


    }
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_info_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}