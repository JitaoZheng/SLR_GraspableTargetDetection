#include "detect_graspable_points/detect_graspable_points.hpp"

// This callback function is just example.
// This callback function is not needed to be executed in loop function...
void cloud_cb(sensor_msgs::PointCloud2ConstPtr input_cloud_msg)
{

  detect_graspable_points detect_graspable_points(input_cloud_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_graspable_points");
  ros::NodeHandle nh("~");


  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, &cloud_cb);
  ros::Subscriber sub = nh.subscribe("/merged_pcd", 1, &cloud_cb);

  ros::spin();
}
