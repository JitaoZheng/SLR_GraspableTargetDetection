#include "detect_graspable_points/detect_graspable_points.hpp"




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectGraspablePoints>());
    rclcpp::shutdown();
    return 0;
}