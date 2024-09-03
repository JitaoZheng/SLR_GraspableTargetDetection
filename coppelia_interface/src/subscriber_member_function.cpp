
#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>   
#include <sstream>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;


class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();

    subscriptionDepth_1.subscribe(this, "/spherical1/depth", rmw_qos_profile);
    subscriptionDepth_2.subscribe(this, "/spherical2/depth", rmw_qos_profile);
    subscriptionDepth_3.subscribe(this, "/spherical3/depth", rmw_qos_profile);
    subscriptionDepth_4.subscribe(this, "/spherical4/depth", rmw_qos_profile);
    subscriptionDepth_5.subscribe(this, "/spherical5/depth", rmw_qos_profile);
    subscriptionDepth_6.subscribe(this, "/spherical6/depth", rmw_qos_profile);
    subscriptionRgb_.subscribe(this, "/spherical1/rgb", rmw_qos_profile);
    subscriptionInfo_.subscribe(this, "/info", rmw_qos_profile);
    
    publisher_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/infosync", qos);
    publisher_depth_1 = this->create_publisher<sensor_msgs::msg::Image>("/depth1sync", qos);
    publisher_depth_2 = this->create_publisher<sensor_msgs::msg::Image>("/depth2sync", qos);
    publisher_depth_3 = this->create_publisher<sensor_msgs::msg::Image>("/depth3sync", qos);
    publisher_depth_4 = this->create_publisher<sensor_msgs::msg::Image>("/depth4sync", qos);
    publisher_depth_5 = this->create_publisher<sensor_msgs::msg::Image>("/depth5sync", qos);
    publisher_depth_6 = this->create_publisher<sensor_msgs::msg::Image>("/depth6sync", qos);
    publisher_rgb_ = this->create_publisher<sensor_msgs::msg::Image>("/rgbsync", qos);
    
    subscriptionDepth_1.registerCallback(std::bind(&MinimalSubscriber::depth1_callback, this, std::placeholders::_1));
    subscriptionDepth_2.registerCallback(std::bind(&MinimalSubscriber::depth2_callback, this, std::placeholders::_1));
    subscriptionDepth_3.registerCallback(std::bind(&MinimalSubscriber::depth3_callback, this, std::placeholders::_1));
    subscriptionDepth_4.registerCallback(std::bind(&MinimalSubscriber::depth4_callback, this, std::placeholders::_1));
    subscriptionDepth_5.registerCallback(std::bind(&MinimalSubscriber::depth5_callback, this, std::placeholders::_1));
    subscriptionDepth_6.registerCallback(std::bind(&MinimalSubscriber::depth6_callback, this, std::placeholders::_1));
    subscriptionRgb_.registerCallback(std::bind(&MinimalSubscriber::rgb_callback, this, std::placeholders::_1)); 
    subscriptionInfo_.registerCallback(std::bind(&MinimalSubscriber::info_callback, this, std::placeholders::_1)); 
    

    timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalSubscriber::SendingFunction, this));
  /* 
    img_sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>(subscriptionDepth_, subscriptionRgb_, subscriptionInfo_, 3);
    img_sync_->registerCallback(std::bind(&MinimalSubscriber::ImgSyncCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  */
  }

private:

  void depth1_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    
    // if callback receives message, depth_flag turns true and the message is given to depth_image_buffer and the
    depth_flag = true;
    depth1_image_buffer = *msg;
    //SendingFunction();


    RCLCPP_INFO(this->get_logger(), "Frame '%s', with ts %u.%u sec ",
                msg->header.frame_id.c_str(),
                msg->header.stamp.sec, 
                msg->header.stamp.nanosec);
  }
  void depth2_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    
    // if callback receives message, depth_flag turns true and the message is given to depth_image_buffer and the
    depth_flag = true;
    depth2_image_buffer = *msg;
    //SendingFunction();

  }
  void depth3_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    
    // if callback receives message, depth_flag turns true and the message is given to depth_image_buffer and the
    depth_flag = true;
    depth3_image_buffer = *msg;
    //SendingFunction();

  }
  void depth4_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    
    // if callback receives message, depth_flag turns true and the message is given to depth_image_buffer and the
    depth_flag = true;
    depth4_image_buffer = *msg;
    //SendingFunction();

  }
  void depth5_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    
    // if callback receives message, depth_flag turns true and the message is given to depth_image_buffer and the
    depth_flag = true;
    depth5_image_buffer = *msg;
    //SendingFunction();

  }
  void depth6_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    
    // if callback receives message, depth_flag turns true and the message is given to depth_image_buffer and the
    depth_flag = true;
    depth6_image_buffer = *msg;
    //SendingFunction();

  }
  void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    
    rgb_flag = true;
    rgb_image_buffer = *msg;
    //SendingFunction();

/*     RCLCPP_INFO(this->get_logger(), "Frame '%s', with ts %u.%u sec ",
                msg->header.frame_id.c_str(),
                msg->header.stamp.sec, 
                msg->header.stamp.nanosec); */
  }
  void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    info_buffer = *msg;
/*     RCLCPP_INFO(this->get_logger(), "Frame '%s', with ts %u.%u sec ",
                msg->header.frame_id.c_str(),
                msg->header.stamp.sec, 
                msg->header.stamp.nanosec); */
  }
  /* 
  void ImgSyncCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& msg_1,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg_2,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_3) const{
                
                RCLCPP_INFO(this->get_logger(),
                "I heard and synchronized the following timestamps: %u, %u, %u",
                msg_1->header.stamp.sec, msg_2->header.stamp.sec, msg_3->header.stamp.sec);
  }
   */
  void SendingFunction(){
      //if (depth_flag & rgb_flag)
      //{
/*         rclcpp::Time now = this->get_clock()->now();
        depth_image_buffer.header.stamp = now;
        rgb_image_buffer.header.stamp = now;
        info_buffer.header.stamp = now; */

        publisher_depth_1->publish(depth1_image_buffer);
        publisher_depth_2->publish(depth2_image_buffer);
        publisher_depth_3->publish(depth3_image_buffer);
        publisher_depth_4->publish(depth4_image_buffer);
        publisher_depth_5->publish(depth5_image_buffer);
        publisher_depth_6->publish(depth6_image_buffer);
        publisher_rgb_->publish(rgb_image_buffer);
        publisher_info_->publish(info_buffer);
        
        // depth_flag = false;
        // rgb_flag = false;

        /* begin = std::chrono::system_clock::now();
        elapsed_seconds = begin - end;
        RCLCPP_INFO(this->get_logger(), to_string(elapsed_seconds.count()));
        end = begin; */
      //} 
  }

  bool depth_flag = false;
  bool rgb_flag = false;

  message_filters::Subscriber<sensor_msgs::msg::Image> subscriptionDepth_1;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriptionDepth_2;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriptionDepth_3;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriptionDepth_4;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriptionDepth_5;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriptionDepth_6;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriptionRgb_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> subscriptionInfo_;
  // std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>> img_sync_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_1;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_2;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_3;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_4;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_5;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_6;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_info_;

  sensor_msgs::msg::Image depth1_image_buffer;
  sensor_msgs::msg::Image depth2_image_buffer;
  sensor_msgs::msg::Image depth3_image_buffer;
  sensor_msgs::msg::Image depth4_image_buffer;
  sensor_msgs::msg::Image depth5_image_buffer;
  sensor_msgs::msg::Image depth6_image_buffer;
  sensor_msgs::msg::Image rgb_image_buffer;
  sensor_msgs::msg::CameraInfo info_buffer;

  rclcpp::TimerBase::SharedPtr timer_;

  std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
  std::chrono::system_clock::time_point end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - begin;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  //auto start_overall = std::chrono::high_resolution_clock::now();

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  
  //auto stop_overall = std::chrono::high_resolution_clock::now();
  //auto duration_overall = std::chrono::duration_cast<std::chrono::microseconds>(stop_overall - start_overall);
  //cout <<"Total time in µs : " << duration_overall.count() << endl;

  rclcpp::shutdown();
  return 0;
}
