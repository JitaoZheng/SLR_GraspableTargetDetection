/*!
 * \file detect_graspable_points.hpp
 * \author Naomasa Takada, Jitao Zheng
 * \brief Describes the header of graspable target detection algorithm.
 */

#ifndef DETECT_GRASPABLE_POINTS_HPP
#define DETECT_GRASPABLE_POINTS_HPP

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cmath>


#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include "tf2_ros/transform_broadcaster.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.hpp>

#include <pcl/surface/mls.h>

#include <pcl/io/pcd_io.h>

#include <libInterpolate/AnyInterpolator.hpp>
#include <libInterpolate/Interpolate.hpp>

#include <chrono>



using namespace std;


class DetectGraspablePoints : public rclcpp::Node
{
public:

    DetectGraspablePoints();
    ~DetectGraspablePoints();


private:

	void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud_msg);

	std::vector<float> getMinValues(const pcl::PointCloud<pcl::PointXYZ>& pointCloud);

	
	void pcd_least_squares_plane(const pcl::PointCloud<pcl::PointXYZ> raw_pcd, Eigen::Vector4f &centroid,  Eigen::Vector3f &normal_vector_of_plane);
	/**
	 * \function name : pcd_least_squares_plane()
	 * \brief : compute centroid point of input point cloud and normal vector of regression plane
	 * \param raw_pcd : input point cloud
	 * \param centroid : coordinate of centroid (output)
	 * \param normal_vector_of_plane : normal vector of regression plane (output)
	 * \return none
	 */	


	
	void pcd_transform ( const pcl::PointCloud<pcl::PointXYZ>  &raw_pcd, pcl::PointCloud<pcl::PointXYZ> &transformed_point_cloud, Eigen::Vector4f &centroid_vector_of_plane, Eigen::Matrix3f &rotation_matrix);
	/**
	 * \function name : pcd_transform()
	 * \brief : transform coordinate of pointcloud to terrain base frame
	 * \param raw_pcd : input point cloud
	 * \param transformed_point_cloud : transormed point cloud (output)
	 * \param centroid_vector_of_plane : coordinate of centroid (output)
	 * \param rotation_matrix : rotation matrix used for the coordinates transformation (output)
	 * \return none
	 */	

	
	void pcd_interpolate (const pcl::PointCloud<pcl::PointXYZ>  raw_pcd,pcl::PointCloud<pcl::PointXYZ> &interpolated_point_cloud);
	/**
	 * \function name : pcd_interpolate()
	 * \brief : homogenize the point cloud and interpolate occlusion points
	 * \param raw_pcd : input point cloud
	 * \param interpolated_point_cloud : interpolated point cloud (output)
	 * \return none
	 */	


	
	vector<vector<vector<int>>> pcd_voxelize (const pcl::PointCloud<pcl::PointXYZ>  input_pcd, const float cube_size);
	/**
	 * \function name : pcd_voxelize()
	 * \brief : turns the point_cloud into a 3D array of 0 and 1, 0 being void and 1 solid
	 * \param input_pcd : input point cloud
	 * \param cube_size : size of the voxel
	 * \return none
	 */	


	
	void downsampling(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg, pcl::PointCloud<pcl::PointXYZ>& output_pcd, const float cube_size);
	/**
	 * \function name : downsampling() 
	 * \brief : execute downsampling to input pointcloud (downsampling means the number of pointcloud is reduced)
	 * \param cloud_msg : pointer of sensor_msgs::msg::PointCloud2 (input)
	 * \param frame_id : reference frame of this point
	 * \param filtered_points : downsampling points (output)
	 * \param cube_size : voxel size
	 * \return none
	 */	

	void tf_broadcast_from_pose(const std::string parant_frame_id, const std::string child_frame_id_to, geometry_msgs::msg::Pose relative_pose_between_frame);
	
	//std::string vox_compare(const int number_of_points, const vector<vector<vector<int>>> subset_of_voxel_array, const vector<vector<vector<int>>> gripper_mask);
	/**
	 * \function name : vox_compare() (not in use!)
	 * \brief : compares two voxel arrays of the same size. If the first voxel array has a solid voxel where the second voxel array has an empty voxel, we conclude that these arrays do not match.
	 * \param subset_of_voxel_array : l*m*n voxelgrid
	 * \param gripper_mask : l*m*n voxelgrid
	 * \return matching_result: "regular","sub", or "no"
	 	"regular" means that subset_of_voxel_array certainly can be grasped. 
		"sub" means that subset_of_voxel_array possibly can be grasped. 
		"no" means that subset_of_voxel_array cannot be grasped. 
	 */	

	
	float vox_evaluate(const int number_of_points, const vector<vector<vector<int>>>& subset_of_voxel_array, const vector<vector<vector<int>>>& gripper_mask);
	/**
	 * \function name : vox_evaluate() 
	 * \brief : compares two voxel arrays of the same size and returns a value between 1 and 100
	 * \param subset_of_voxel_array : l*m*n voxelgrid
	 * \param gripper_mask : l*m*n voxelgrid
	 * \return graspability: the probability of graspability of the respective point
	 */	

	
	vector<vector<vector<int>>> vox_clip(const int x, const int y, vector<vector<vector<int>>> search_voxel_array);
	/**
	 * \function name : vox_clip() 
	 * \brief : clips the searched array at all sides by setting the values to zero. 
	 We want to prevent the gripper mask from protruding from the voxel array during the matching algorithm. 
	 We can limit the range of searching by using this function because the values in the area around the outer edges are set 0. 
	 Then we will compare  3-dimensional arrays based on solid(1) voxels located in the inside area having room for gripper-mask.
	 * \param x: number of surfaces set 0 in x-direction
	 * \param y : number of surfaces set 0 in y-direction
	 * \param search_voxel_array : search voxelgrid
	 * \return searchVoxelArray: search voxelgrid having the room of a gripper mask around the outer edges
	 */	

	
	vector<vector<vector<int>>> vox_extract(const vector<vector<vector<int>>>& voxel_array ,const vector<int>& position_reference_point, const vector<int>& size_extracting, const float palm_diameter, const float closing_angle, const float voxel_size);
 	/**
	 * \function name : vox_extract() 
	 * \brief : extracts an array of voxels of the specified size from a larger voxel array.
	 We want to compare the gripper mask with the extracted voxel array of the same size as the gripper mask.
	 * \param voxel_array : m*n*l matrix, 3-dimensional voxelgrid 
	 * \param position_reference_point : 1*3 vector, position of the extracted voxel array, xyz 
	 * \param size_extracting: 1*3 vector, size of the of the extracted voxelgrid ,i*i*j
	 * \return extractev_voxel_array: i*i*j matrix, extracted 3-dimensional voxel array (i*i*j matrix)
	 */	

	
	struct GripperParam{
		float palm_diameter;
		float palm_diameter_of_finger_joints;
		float finger_length;
		float spine_length;
		float spine_depth;
		float opening_angle;
		float closing_angle;
		float opening_spine_radius; // the distance from the center of palm to the farthest point of the spines
		float opening_spine_depth;
		float closing_height; // Vertical distance between the tip of the spine and the bottom of the palm when closed
		float margin_of_top_solid_diameter;
		float inside_margin_of_bottom_void_diameter;
	};

	struct Subscripts {
    std::vector<int> x;
    std::vector<int> y;
    std::vector<int> z;
	};

	
	
	struct MatchingSettings
	{	
		float voxel_size;
		int threshold;
		std::string delete_lower_targets;
		float delete_lower_targets_threshold;
		std::string interpolation;
		std::string peaks; // peaks or not
		float searching_radius_for_normal_and_curvature;
		int extra_sheet;
		float graspability_threshold;
	};

	vector<vector<vector<int>>> creategrippermask(GripperParam gripper_param, float voxel_size, const MatchingSettings& matching_settings);
	/**
	 * \function name : gripper_mask() 
	 * \brief : makes the gripper_mask, which is the 3-dimensional array composed of 0 and 1 considering geometric parameters of the gripper.
	 * \param voxel_size: length of one side of voxel [m]
	 * \param gripper_param : geometric parameters of the gripper which are set in the config file.
	 * \param matching_settings : Matching settings
	 * \return gripper_mask : n*n*m array composed of 0 and 1.
	 */	


	
	void detectTerrainPeaks(pcl::PointCloud<pcl::PointXYZ> input_cloud, sensor_msgs::msg::PointCloud2 &cloud_msg, const MatchingSettings& matching_settings);
	/**
	 * \function name : detectTerrainPeaks()
	 * \brief : estimate the normal vectors of each point and compute the curvature, returns the points with the largest principal curvature eigenvalue
	 * \param input_cloud : input point cloud
	 * \param cloud_msg : ROS pointcloud message
	 * \return none
	 */	



	vector<vector<int>> voxel_matching(vector<vector<vector<int>>>& terrain_matrix, const vector<vector<vector<int>>>& gripper_mask, const MatchingSettings& matching_settings, GripperParam gripper_param);
	/**
	 * \function name : voxel_matching() 
	 * \brief : finds subset voxel arrays in a major voxel array that matches a gripper mask voxel array.
	 * \param terrain_matrix : 3-dimensional array composed of 0 and 1
	 * \param gripper_mask : 3-dimensional array of gripper mask composed of 0 and 1
	 * \param matching_settings : structure parameters of detection settings set in the config file
	 * \return voxel_array_of_graspable_points : 4*n matrix, containing subscripts of "grippable" points and "grippability" at those points  
                        						1st,2nd, and 3rd rows indicate subscripts in x-y-z directionss
                                     			4th row indicates the number of solid
                                     			voxels, but that of the sub-graspable
                                     			points is 1
	 */	



	std::vector<std::vector<float>> pcd_re_transform(std::vector<std::vector<int>> voxel_coordinates_of_graspable_points, float voxel_size, std::vector<float> offset_vector, Eigen::Matrix3f rotation_matrix, Eigen::Vector4f centroid_vector_of_plane);
													//Eigen::Matrix3f rotation_matrix, 
													//Eigen::Vector4f centroid_vector_of_plane, 
													//std::vector<float> offset_vector
													
	/**
	 * \function name : pcd_re_transform() 
	 * \brief : returns the coordinates of the voxel array to the original input coordinate system
	 * \param voxel_coordinates_of_graspable_points : 4*n matrix. Each column indicates a matched voxel. 1st, 2nd, and 3rd raws indicate x y z subscripts in voxel array.
	 * \param voxel_size : length of one side of voxel used in voxelize [m]
	 * \param rotation_matrix
	 * \param centroid_vector_of_plane
	 * \return graspable_points: std::vector<std::vector<float>>
	 */	



	sensor_msgs::msg::PointCloud2 visualizeRainbow(std::vector<std::vector<float>> array, const MatchingSettings& matching_settings);

	sensor_msgs::msg::PointCloud2 combinedAnalysis(const std::vector<std::vector<float>> array, const pcl::PointCloud<pcl::PointXYZRGB> cloud2, float distance_threshold, const MatchingSettings& matching_settings);
	
	visualization_msgs::msg::MarkerArray visualizeGScore(std::vector<std::vector<float>> array, const MatchingSettings& matching_settings);

	rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_point_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr interpolate_point_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr peaksPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rainbowPub;
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combinedPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr g_score_pub;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rawPcdSubscriber_;


	std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf;
};


#endif
