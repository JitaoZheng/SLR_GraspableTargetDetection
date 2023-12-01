
#include <iostream> 
#include <fstream> 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>


using namespace std;
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pointcloud_publisher");
    ros::NodeHandle nh;

    // Create a publisher for the point cloud
    ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2>("merged_pcd", 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("peaks_cloud", 1);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {   

        // Create a point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> peaks;
        vector<int> indices;

        // ****** SCAR-E Maps (Large maps, set voxel size to 0.01!) ******* //

        // ****** Scene 1 ******* //

        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/scanned_cloud_primitive.pcd",cloud);

        // ****** Scene 2 ******* //

        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/simulated_cloud.pcd",cloud);
        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/simulated_cloud_pre-scan.pcd",cloud);

        // ****** Scene 3 ******* //

        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/scanned_cloud_Apr8.pcd",cloud);

        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/testfield_apr8_upscaled.pcd",cloud);


        // ****** Scene 4 ******* //

        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/scanned_cloud_realtime_1.5x.pcd",cloud);
        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/Artificial_rocks_pose_1.pcd",cloud);
        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/artificial_rocks.pcd",cloud);
        //pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/scanned_cloud_realtime.pcd",cloud);

        // ****** HubRobo Maps (small maps, set voxel size to 0.001!) ******* //

        pcl::io::loadPCDFile<pcl::PointXYZ>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/leaning_bouldering_holds.pcd",cloud);


        // ****** Cloud of curvatures ******* //
        // Remove that if the bug in curvature detection is fixed

        pcl::io::loadPCDFile<pcl::PointXYZRGB>("src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/peak_pcd.pcd",peaks);

        // Convert the point cloud to a ROS message
        sensor_msgs::PointCloud2 output1;
        sensor_msgs::PointCloud2 output2;
        pcl::toROSMsg(cloud, output1);
        pcl::toROSMsg(peaks, output2);

        // Set the header information
        output1.header.frame_id = "regression_plane_frame";
        output1.header.stamp = ros::Time::now();

        output2.header.frame_id = "regression_plane_frame";
        output2.header.stamp = ros::Time::now();

        // Publish the point cloud
        pub1.publish(output1);
        pub2.publish(output2);

        cout <<"PointCloud published under the frame'camera_depth_optical_frame'" << endl;

        // Sleep to maintain the 1 Hz publishing rate
        loop_rate.sleep();
    }


    return 0;


   
}