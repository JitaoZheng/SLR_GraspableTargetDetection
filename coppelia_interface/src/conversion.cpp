#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>


#include "depthimage_to_pointcloud2/depth_conversions.hpp"
#include <image_geometry/pinhole_camera_model.h>
#include "rclcpp/rclcpp.hpp"
#include <rcutils/logging_macros.h>
#include <sensor_msgs/image_encodings.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/io/concatenate_data.hpp>

// #include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/filter.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>


// using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;

// static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_point_cloud;


static sensor_msgs::msg::CameraInfo::SharedPtr cam_info;
// static const std::string OPENCV_WINDOW = "Image window";

// cam_info = camera_info_filler(cam_info);


class DepthimageToPointCloud2 : public rclcpp::Node
{
public:
    DepthimageToPointCloud2() : Node("depthimage_to_pointcloud2")
    {
        rclcpp::QoS qos(10);
        // auto rmw_qos_profile = qos.get_rmw_qos_profile();
/* 
        rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

        custom_qos_profile.depth = 1;
        custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
 */
        infoSubscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/infosync", 10, std::bind(&DepthimageToPointCloud2::info_callback, this, std::placeholders::_1));
        depthSubscriber_1 = this->create_subscription<sensor_msgs::msg::Image>("/depth1sync", 10, std::bind(&DepthimageToPointCloud2::depth_callback_1, this, std::placeholders::_1));
        depthSubscriber_2 = this->create_subscription<sensor_msgs::msg::Image>("/depth2sync", 10, std::bind(&DepthimageToPointCloud2::depth_callback_2, this, std::placeholders::_1));
        depthSubscriber_3 = this->create_subscription<sensor_msgs::msg::Image>("/depth3sync", 10, std::bind(&DepthimageToPointCloud2::depth_callback_3, this, std::placeholders::_1));
        depthSubscriber_4 = this->create_subscription<sensor_msgs::msg::Image>("/depth4sync", 10, std::bind(&DepthimageToPointCloud2::depth_callback_4, this, std::placeholders::_1));
        depthSubscriber_5 = this->create_subscription<sensor_msgs::msg::Image>("/depth5sync", 10, std::bind(&DepthimageToPointCloud2::depth_callback_5, this, std::placeholders::_1));
        depthSubscriber_6 = this->create_subscription<sensor_msgs::msg::Image>("/depth6sync", 10, std::bind(&DepthimageToPointCloud2::depth_callback_6, this, std::placeholders::_1));
        //depthSubscriber_.subscribe(this, "/depthsync", custom_qos_profile);
        //infoSubscriber_.subscribe(this, "/infosync", custom_qos_profile);

        //depthSubscriber_.registerCallback(std::bind(&DepthimageToPointCloud2::depth_callback, this, std::placeholders::_1));
        //infoSubscriber_.registerCallback(std::bind(&DepthimageToPointCloud2::info_callback, this, std::placeholders::_1));
        
        g_pub_point_cloud_1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/converted_pcd_1", qos);
        /* 
        g_pub_point_cloud_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/converted_pcd_2", qos);
        g_pub_point_cloud_3 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/converted_pcd_3", qos);
        g_pub_point_cloud_4 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/converted_pcd_4", qos);
        g_pub_point_cloud_5 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/converted_pcd_5", qos);
        g_pub_point_cloud_6 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/converted_pcd_6", qos);
        */
        merged_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_pcd", qos);

        timer_ = this->create_wall_timer(1000ms, std::bind(&DepthimageToPointCloud2::SendingFunction, this));



    }

private:
    void depth_callback_1(const sensor_msgs::msg::Image::ConstSharedPtr imageraw)
    {          
        cloud_buffer_1 = depth_conversion(imageraw);
        cloud_buffer_1 = pcd_transformation(cloud_buffer_1, 1);
    }
    void depth_callback_2(const sensor_msgs::msg::Image::ConstSharedPtr imageraw)
    {   
        cloud_buffer_2 = depth_conversion(imageraw);
        cloud_buffer_2 = pcd_transformation(cloud_buffer_2, 2);
    }
    void depth_callback_3(const sensor_msgs::msg::Image::ConstSharedPtr imageraw)
    {   
        cloud_buffer_3 = depth_conversion(imageraw);
        cloud_buffer_3 = pcd_transformation(cloud_buffer_3, 3);
    }
    void depth_callback_4(const sensor_msgs::msg::Image::ConstSharedPtr imageraw)
    {   
        cloud_buffer_4 = depth_conversion(imageraw);
        cloud_buffer_4 = pcd_transformation(cloud_buffer_4, 4);
    }
    void depth_callback_5(const sensor_msgs::msg::Image::ConstSharedPtr imageraw)
    {   
        cloud_buffer_5 = depth_conversion(imageraw);
        cloud_buffer_5 = pcd_transformation(cloud_buffer_5, 5);
    }
    void depth_callback_6(const sensor_msgs::msg::Image::ConstSharedPtr imageraw)
    {   
        cloud_buffer_6 = depth_conversion(imageraw);
        cloud_buffer_6 = pcd_transformation(cloud_buffer_6, 6);
    }
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr info)
    {
        RCLCPP_INFO(this->get_logger(), "Camera Info received.");
        cam_info = info;
    }
    sensor_msgs::msg::PointCloud2 depth_conversion(const sensor_msgs::msg::Image::ConstSharedPtr imageraw)
    {   
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        if (nullptr == cam_info) {
            // we haven't gotten the camera info yet, so just drop until we do
            RCUTILS_LOG_WARN("No camera info, skipping point cloud conversion");
            
            return *cloud_msg;
        }

        
        cloud_msg->header = imageraw->header;
        cloud_msg->height = imageraw->height;
        cloud_msg->width = imageraw->width;
        cv_bridge::CvImagePtr image;
        
        
        try
        {
            image = cv_bridge::toCvCopy(imageraw, sensor_msgs::image_encodings::MONO16);
        }
        catch (cv_bridge::Exception& e)
        {
            RCUTILS_LOG_WARN("cv_bridge exception: %s", e.what());
            return *cloud_msg;
        }
        sensor_msgs::msg::Image::ConstSharedPtr msg_16 = image->toImageMsg();
        


        //image = cv_bridge::CvImage.toImageMsg();

        cloud_msg->is_dense =false;
        cloud_msg->is_bigendian = false;
        cloud_msg->fields.clear();
        cloud_msg->fields.reserve(1);

        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        // cam_info here is a sensor_msg::msg::CameraInfo::SharedPtr,
        // which we get from the depth_camera_info topic.
        image_geometry::PinholeCameraModel model;
        model.fromCameraInfo(cam_info);

        if (msg_16->encoding == sensor_msgs::image_encodings::MONO16) {
            depthimage_to_pointcloud2::convert<uint16_t>(msg_16, cloud_msg, model);
        } else if (msg_16->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            depthimage_to_pointcloud2::convert<float>(msg_16, cloud_msg, model);
        } else {
            RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 5000,
            "Depth image has unsupported encoding [%s]", msg_16->encoding.c_str());
            return *cloud_msg;
        }

        return *cloud_msg;
        
    }

    sensor_msgs::msg::PointCloud2 pcd_transformation(const sensor_msgs::msg::PointCloud2 pcd, int cam_number)
    {
        /* Reminder: how transformation matrices work :

                    |-------> This column is the translation
            | 1 0 0 x |  \
            | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
            | 0 0 1 z |  /
            | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

        */
        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
        // Reference point is camera #1
        //    (row, column)
        float alpha = -M_PI/2; // The angle of rotation around x in radians
        float gamma = 0; // The angle of rotation around z in radians
        switch(cam_number){
            case 1:{
                float beta = 0; // The angle of rotation around z in radians
                transform_1 = AngularTransformation(transform_1, alpha, beta, gamma);
                transform_1 (0,3) = 0;
                transform_1 (1,3) = 0.2;
                break;
            }
            
            case 2:{
                float beta = M_PI/3; // The angle of rotation around z in radians
                transform_1 = AngularTransformation(transform_1, alpha, beta, gamma);
                transform_1 (0,3) = -0.1732;
                transform_1 (1,3) = 0.1;
                break;
            }

            case 3:{
                float beta = 2*M_PI/3; // The angle of rotation around z in radians
                transform_1 = AngularTransformation(transform_1, alpha, beta, gamma);
                transform_1 (0,3) = -0.1732;
                transform_1 (1,3) = -0.1;
                break;
            }

            case 4:{
                float beta = M_PI; // The angle of rotation around z in radians
                transform_1 = AngularTransformation(transform_1, alpha, beta, gamma);
                transform_1 (0,3) = 0;
                transform_1 (1,3) = -0.2;
                break;
            }

            case 5:{
                float beta = 4*M_PI/3; // The angle of rotation around z in radians
                transform_1 = AngularTransformation(transform_1, alpha, beta, gamma);
                transform_1 (0,3) = 0.1732;
                transform_1 (1,3) = -0.1;
                break;
            }

            case 6:{
                float beta = 5*M_PI/3; // The angle of rotation around z in radians
                transform_1 = AngularTransformation(transform_1, alpha, beta, gamma);
                transform_1 (0,3) = 0.1732;
                transform_1 (1,3) = 0.1;
                break;
            }

            default:{
                cout << "Error! The operator is not correct";
                
                break;
            }
        }
                
        
        

        // Executing the transformation
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        // You can either apply transform_1 or transform_2; they are the same
        pcl_ros::transformPointCloud (transform_1, pcd, transformed_cloud);

        return transformed_cloud;
    }

    Eigen::Matrix4f AngularTransformation(Eigen::Matrix4f transform_1, float alpha, float beta, float gamma)
    {

        transform_1 (0,0) = cos (beta)*cos (gamma);
        transform_1 (0,1) = -cos(beta)*sin(gamma);
        transform_1 (0,2) = -sin(beta);
        transform_1 (1,0) = -sin(alpha)*sin(beta)*cos(gamma)+cos(alpha)*sin(gamma);
        transform_1 (1,1) = sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma);
        transform_1 (1,2) = -sin(alpha)*cos(beta);
        transform_1 (2,0) = cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
        transform_1 (2,1) = -cos(alpha)*sin(beta)*sin(gamma)+sin(alpha)*cos(gamma);
        transform_1 (2,2) = cos(alpha)*cos(beta);
        
/* 
        transform_1 (0,0) = cos (beta)*cos (gamma);
        transform_1 (0,1) = -cos(beta)*sin(gamma);
        transform_1 (0,2) = -sin(beta);
        transform_1 (1,0) = -sin(alpha)*sin(beta)*cos(gamma)+cos(alpha)*sin(gamma);
        transform_1 (1,1) = sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma);
        transform_1 (1,2) = -sin(alpha)*cos(beta);
        transform_1 (2,0) = cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
        transform_1 (2,1) = -cos(alpha)*sin(beta)*sin(gamma)+sin(alpha)*cos(gamma);
        transform_1 (2,2) = cos(alpha)*cos(beta);
         */
        return transform_1;
    }

    void CombineClouds(const sensor_msgs::msg::PointCloud2 & in1, const sensor_msgs::msg::PointCloud2 & in2, sensor_msgs::msg::PointCloud2 & out)
    {
        pcl::PCLPointCloud2 in1_t;
        pcl::PCLPointCloud2 in2_t;
        pcl::PCLPointCloud2 out_t;
        pcl_conversions::toPCL(in1,in1_t);
        pcl_conversions::toPCL(in2,in2_t);
        // Concatenate the results
        pcl::concatenate(in1_t, in2_t, out_t);
        pcl_conversions::fromPCL(out_t, out);  
        // Copy header
        out.header.stamp = in1.header.stamp;
    }

    void CloudsInput(
        const sensor_msgs::msg::PointCloud2 in1, const sensor_msgs::msg::PointCloud2 in2,
        const sensor_msgs::msg::PointCloud2 in3, const sensor_msgs::msg::PointCloud2 in4,
        const sensor_msgs::msg::PointCloud2 in5, const sensor_msgs::msg::PointCloud2 in6)
    {
        sensor_msgs::msg::PointCloud2 out1;
        sensor_msgs::msg::PointCloud2 out2;
        CombineClouds(in1, in2, out1);

        CombineClouds(out1, in3, out2);
        out1 = out2;

        CombineClouds(out2, in4, out1);

        CombineClouds(out1, in5, out2);
        out1 = out2;

        CombineClouds(out2, in6, out1);
        merged_pcd_buffer = out1;

    }

    void SendingFunction()
    {
        
        g_pub_point_cloud_1->publish(cloud_buffer_1);
        /* 
        g_pub_point_cloud_2->publish(cloud_buffer_2);
        g_pub_point_cloud_3->publish(cloud_buffer_3);
        g_pub_point_cloud_4->publish(cloud_buffer_4);
        g_pub_point_cloud_5->publish(cloud_buffer_5);
        g_pub_point_cloud_6->publish(cloud_buffer_6);
        // cam_info = info_buffer;
        RCLCPP_INFO(this->get_logger(), "Point Cloud created and published.");
         */
        CloudsInput(cloud_buffer_1, cloud_buffer_2, cloud_buffer_3, cloud_buffer_4, cloud_buffer_5, cloud_buffer_6);
        // Set the header information
        //merged_pcd_buffer.header.frame_id = "world";
        //merged_pcd_buffer.header.stamp = this->get_clock()->now();
        merged_cloud->publish(merged_pcd_buffer);
        pcl::fromROSMsg (merged_pcd_buffer, cloud);

        vector<int> indices;
        pcl::removeNaNFromPointCloud(cloud, cloud, indices);
        // writer.write<pcl::PointXYZRGB> ("/home/jitao/Documents/catkin_ws/src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/realtime.pcd", cloud, false);
        
        numbering += 1;
        //pcl::io::savePCDFileASCII("/home/jitao/catkin_ws/src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/meshlab_icp/cloud_"+std::to_string(numbering)+".pcd", cloud);
        //pcl::io::savePCDFileASCII("/home/jitao/catkin_ws/src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/circular_cloud_F_0_-1.5.pcd", cloud);
        RCLCPP_INFO(this->get_logger(), "Merged point cloud created and published.");
        

    }


    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_point_cloud_1;
    /* 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_point_cloud_2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_point_cloud_3;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_point_cloud_4;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_point_cloud_5;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_point_cloud_6;
    */

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud;
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info;
    
    sensor_msgs::msg::PointCloud2 cloud_buffer_1;
    sensor_msgs::msg::PointCloud2 cloud_buffer_2;
    sensor_msgs::msg::PointCloud2 cloud_buffer_3;
    sensor_msgs::msg::PointCloud2 cloud_buffer_4;
    sensor_msgs::msg::PointCloud2 cloud_buffer_5;
    sensor_msgs::msg::PointCloud2 cloud_buffer_6;
    sensor_msgs::msg::PointCloud2 merged_pcd_buffer;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PCDWriter writer;
    sensor_msgs::msg::CameraInfo info_buffer;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSubscriber_1;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSubscriber_2;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSubscriber_3;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSubscriber_4;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSubscriber_5;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSubscriber_6;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr infoSubscriber_;
    int numbering = 1;

};

int main(int argc, char * argv[])
{
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthimageToPointCloud2>());
    rclcpp::shutdown();
    return 0;
}