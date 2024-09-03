#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class PointCloudRegistration
{
public:
    PointCloudRegistration() : viewer("ICP Viewer")
    {
        cumulative_transform.setIdentity();
        initial_position_set = false;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        PointCloud input_cloud;
        pcl::fromROSMsg(*msg, input_cloud);

        // If it's the first cloud, just store it and set the initial position
        if (!prev_cloud_loaded)
        {
            prev_cloud = input_cloud;
            prev_cloud_loaded = true;

            // Set initial position based on the received messages
            setInitialPosition();
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
        cumulative_transform *= icp.getFinalTransformation();

        // Concatenate the transformed cloud with the previous cloud
        pcl::transformPointCloud(input_cloud, transformed_cloud, icp.getFinalTransformation());
        merged_cloud += transformed_cloud;

        // Publish the merged cloud
        sensor_msgs::PointCloud2 merged_msg;
        pcl::toROSMsg(merged_cloud, merged_msg);
        merged_msg.header = msg->header;
        merged_cloud_pub.publish(merged_msg);

        // Update the previous cloud for the next iteration
        prev_cloud = merged_cloud;
    }

    void setInitialPosition()
    {
        // Set initial position based on the received messages
        initial_transform.setIdentity();

        if (initial_position_set)
        {
            initial_transform(0, 3) = initial_position.x;
            initial_transform(1, 3) = initial_position.y;
            initial_transform(2, 3) = initial_position.z;
        }

        initial_position_set = true;
    }

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

    void positionCallback(const geometry_msgs::Vector3ConstPtr &position_msg)
    {
        current_position = *position_msg;
    }

    void orientationCallback(const geometry_msgs::QuaternionConstPtr &orientation_msg)
    {
        current_orientation = *orientation_msg;
    }

    void run()
    {
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/your_point_cloud_topic", 1, &PointCloudRegistration::cloudCallback, this);
        ros::Subscriber position_sub = nh.subscribe<geometry_msgs::Vector3>("/your_position_topic", 1, &PointCloudRegistration::positionCallback, this);
        ros::Subscriber orientation_sub = nh.subscribe<geometry_msgs::Quaternion>("/your_orientation_topic", 1, &PointCloudRegistration::orientationCallback, this);
        merged_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/merged_point_cloud", 1);

        ros::spin();
    }

private:
    bool prev_cloud_loaded = false;
    PointCloud prev_cloud;
    PointCloud merged_cloud;
    pcl::visualization::CloudViewer viewer;
    ros::Publisher merged_cloud_pub;

    Eigen::Matrix4f cumulative_transform;
    bool initial_position_set;
    Eigen::Matrix4f initial_transform;
    geometry_msgs::Vector3 initial_position;
    geometry_msgs::Vector3 current_position;
    geometry_msgs::Quaternion current_orientation;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_registration_node");
    PointCloudRegistration registration;
    registration.run();

    return 0;
}
