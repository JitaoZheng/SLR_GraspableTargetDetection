/*!
  * \file detect_graspable_points.hpp
  * \author 
  * Jitao Zheng (jitao.zheng@tum.de)
  * Ringeval-Meusnier Antonin (ringeval-meusnier.antonin.camille.charles.s3@dc.tohoku.ac.jp)
  * Taku Okawara (taku.okawara.t3 at dc.tohoku.ac.jp)
  * Kentaro Uno (unoken at astro.mech.tohoku.ac.jp)
  * \brief Every function of the graspable target detection algorithm
*/

#include "detect_graspable_points/detect_graspable_points.hpp"
#include <pcl/filters/filter.h>
//#include <unistd.h>
//#include <stdio.h>


/*! ******************************
 ***       Constructor       *****
 *********************************/

detect_graspable_points::detect_graspable_points(sensor_msgs::PointCloud2ConstPtr cloud_msg)
{
  // ****** PARAMETERS ******* //

  // ****** SCAR-E Gripper ******* //

  GripperParam gripper_param = { //in [mm]!
		71, // palm_diameter
		92, // palm_diameter_of_finger_joints
		40, // finger_length
		41, // spine_length
		5, // spine_depth
		80, // opening_angle
		10, // closing_angle
		136, // opening_spine_radius, the distance from the center of palm to the furthest point of the spines
		5, // opening_spine_depth
		90, // closing_height, Vertical distance between the tip of the spine and the bottom of the palm when closed
		4, // margin_of_top_solid_diameter
		2, // inside_margin_of_bottom_void_diameter
	}; 

  // ****** HubRobo Gripper ******* //
/* 
  GripperParam gripper_param = { //in [mm]!
		32, // palm_diameter
		28, // palm_diameter_of_finger_joints
		15, // finger_length
		15, // spine_length
		5, // spine_depth
		75, // opening_angle
		30, // closing_angle
		37, // opening_spine_radius, the distance from the center of palm to the furthest point of the spines
		5, // opening_spine_depth
		16, // closing_height, Vertical distance between the tip of the spine and the bottom of the palm when closed
		4, // margin_of_top_solid_diameter
		2, // inside_margin_of_bottom_void_diameter
	};
 */
  // ****** General Parameters ******* //

  MatchingSettings matching_settings ={
    0.005, // voxel size [m]
    120, // threshold of numbers of solid voxels within the subset (TSV) (SCAR-E: 120)
    "off", // delete the targets whose z positions are lower than a limit value: on or off
    -0.05, // [m] Lower threshold of targets (Apr8_realtime: 0.025, Artificial rocks: -0.05, slopes: -0.07, primitive: 0.01, leaning_bouldering_holds: 0.01)
    "on",  //interpolation: "on" or "off"
    "off",  //trying to find the peaks of the terrain? on or off 
    0.09, // [m] Searching radius for the curvature (SCAR-E: 0.09, HubRobo: 0.03)
    3, // size of extra sheet above the top layer of gripper mask (H_add)(SCAR-E: 1)
    80 // Graspability threshold. Above which we can call it graspable with great certainty
  };


  // ************************** //
  
  cout << "start processing" << endl;
  //printf("Current working dir: %s\n", get_current_dir_name());

  auto start_overall = std::chrono::high_resolution_clock::now();
  
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // ****** Downsampling ******* //
  // uncomment this line if you want to proceed with the pcd received as ROS message
  downsampling(cloud_msg, "downsampling_frame", cloud, matching_settings.voxel_size);

  // downsampling (make voxel) is applied to pointcloud.
  // downsampling means the number of pointcloud is reduced
  // voxel means the distance between each point is equal.

  // ****** remove all the NaNs from pcd ******* //
  vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud, cloud, indices);

  // ****** get the min values from the pcd, for offset re-transform ******* //
  /* 
  std::vector<float> min_values_of_original_pcd;
  min_values_of_original_pcd = getMinValues(cloud);

  cout << "Minimum X: " << min_values_of_original_pcd[0] << endl;
  cout << "Minimum Y: " << min_values_of_original_pcd[1] << endl;
  cout << "Minimum Z: " << min_values_of_original_pcd[2] << endl;

  */


  
  
  if(cloud.points.size() > 0) // If downsampled_points.points.size()==0, process will have segmentation fault
  {

    // ****** Transform ******* //
    Eigen::Vector4f centroid_point;
    Eigen::Vector3f normal_vector;
    Eigen::Matrix3f rotation_matrix;
    pcl::PointCloud<pcl::PointXYZ> new_pcl;

    auto start_transform = std::chrono::high_resolution_clock::now();

    pcd_transform(cloud,new_pcl,centroid_point,rotation_matrix);  // /!\ modified downsampled_points to real cloud data 

    auto stop_transform = std::chrono::high_resolution_clock::now();
    auto duration_transform = std::chrono::duration_cast<std::chrono::microseconds>(stop_transform - start_transform);

    cout <<"Time for pcd_transform in µs : " << duration_transform.count() << endl;

    

    // ****** Testing and Debug of Transformation ******* //

    // The following descriotion are used for just visualization of pcd_transform()
    // Just instisute the relative pose between the camera and the regression_plane calculated by pcd_transform()
    geometry_msgs::Pose relative_pose_between_camera_and_regression_plane;
    relative_pose_between_camera_and_regression_plane.position.x = centroid_point[0];
    relative_pose_between_camera_and_regression_plane.position.y = centroid_point[1];
    relative_pose_between_camera_and_regression_plane.position.z = centroid_point[2];
    Eigen::Quaternionf relative_quat_between_camera_and_regression_plane(rotation_matrix);
    //cout << "eigen rotation_matrix = " << endl << rotation_matrix << endl << endl << endl;
    // considering normalization for avoiding an error
    relative_pose_between_camera_and_regression_plane.orientation.x = relative_quat_between_camera_and_regression_plane.normalized().x();
    relative_pose_between_camera_and_regression_plane.orientation.y = relative_quat_between_camera_and_regression_plane.normalized().y();
    relative_pose_between_camera_and_regression_plane.orientation.z = relative_quat_between_camera_and_regression_plane.normalized().z();
    relative_pose_between_camera_and_regression_plane.orientation.w = relative_quat_between_camera_and_regression_plane.normalized().w();
    //cout << "geometry_msgs" << relative_pose_between_camera_and_regression_plane << endl << endl;

    // set a frame between a camera and the regression_plane calculated by pcd_transform()
    tf_broadcast_from_pose("camera_depth_optical_frame", "regression_plane_frame", relative_pose_between_camera_and_regression_plane);

    // publish the 3D centroid point for debug
    visualizePoint(centroid_point[0], centroid_point[1], centroid_point[2], "centroid_point", "camera_depth_optical_frame");
  
    // puvlish the normal vector for debug
    Eigen::Vector3f cetroid_point_3f;  
    cetroid_point_3f << centroid_point[0], centroid_point[1], centroid_point[2];
    // finish the description related to pcd_transform().

    // **************************** //

    vector<vector<vector<int>>> voxel_matrix;  // important that size needs to be 0 in declaration for the resize to work properly
    std::vector<float> offset_vector_for_retransform;


    if (matching_settings.interpolation == "on") {
        // ****** Interpolate ******* //
        pcl::PointCloud<pcl::PointXYZ> pcl_for_testing_interpolation;

        auto start_interp = std::chrono::high_resolution_clock::now();

        pcd_interpolate(new_pcl,pcl_for_testing_interpolation);   

        auto stop_interp = std::chrono::high_resolution_clock::now();
        auto duration_interp = std::chrono::duration_cast<std::chrono::microseconds>(stop_interp - start_interp);

        cout <<"Time for pcd_interpolate in µs : " << duration_interp.count() << endl;

        // publish the pointcloud with respect to regrssion_plane_frame
        sensor_msgs::PointCloud2 interpolated_pointCloud;  // please change
        pcl::toROSMsg(pcl_for_testing_interpolation, interpolated_pointCloud);  // new_pcl -> pcl_for_testing_interpolation
        interpolated_pointCloud.header.frame_id="regression_plane_frame";
        interpolated_pointCloud.header.stamp = ros::Time::now();
        interpolate_point_pub.publish(interpolated_pointCloud);

        
        // ****** Find the peaks ******* //

        // if convex peak detection is set on
        // TODO: there is a bug in this function. The loop crashes as soon as this function finishes. Thus, as a preliminary solution,
        // we save the pcd as a file and go on processing with curvature detection turned "off".
        if (matching_settings.peaks == "on") {

          sensor_msgs::PointCloud2 peak_coordinates;
        
          cout <<"Start to find all peaks within the terrain map."<< endl;
          detectTerrainPeaks(pcl_for_testing_interpolation, peak_coordinates, matching_settings);
          cout <<"finished detecting."<< endl;


          //peaksPub.publish(peak_coordinates);
        }

        // ****** Voxelize ******* //
        auto start_voxel = std::chrono::high_resolution_clock::now();

        voxel_matrix = pcd_voxelize(pcl_for_testing_interpolation, matching_settings.voxel_size);

        auto stop_voxel = std::chrono::high_resolution_clock::now();
        auto duration_voxel = std::chrono::duration_cast<std::chrono::microseconds>(stop_voxel - start_voxel);

        cout <<"Time for pcd_voxelize in µs : " << duration_voxel.count() << endl;

        //get minimum values for re-transform later
        offset_vector_for_retransform = getMinValues(pcl_for_testing_interpolation);


    }
    else {
      // ****** If the Interpolation step is turned off ******* //

      // ****** Find the peaks ******* //
      // TODO: there is a bug in this function. The loop dies as soon as this function finishes. Thus, as a preliminary solution,
      // we save the pcd as a file and go on processing with curvature detection turned "off".
        if (matching_settings.peaks == "on") {

          sensor_msgs::PointCloud2 peak_coordinates;
        
          cout <<"Start to find all peaks within the terrain map."<< endl;
          detectTerrainPeaks(new_pcl, peak_coordinates, matching_settings);
          cout <<"finished detecting."<< endl;


          peaksPub.publish(peak_coordinates);
        }

        // ****** Voxelize ******* //
        
        auto start_voxel = std::chrono::high_resolution_clock::now();
    
        voxel_matrix = pcd_voxelize(new_pcl,matching_settings.voxel_size);

        auto stop_voxel = std::chrono::high_resolution_clock::now();
        auto duration_voxel = std::chrono::duration_cast<std::chrono::microseconds>(stop_voxel - start_voxel);

        cout <<"Time for pcd_voxelize in µs : " << duration_voxel.count() << endl;

        //get minimum values for re-transform later
        offset_vector_for_retransform = getMinValues(new_pcl);
    }

  

    // ****** Gripper mask ******* //

    //create an empty 3d array ready to use for gripper mask
    vector<vector<vector<int>>> gripper_mask(0, std::vector<std::vector<int>>(0, std::vector<int>(0,0)));

    gripper_mask = creategrippermask(gripper_param, matching_settings.voxel_size, matching_settings);
    
    // ****** Voxel matching ******* //

    // create an empty 2d array ready to use. it will be 4 columns: x, y, z and graspability score
    vector<vector<int>> graspable_points(0, std::vector<int>(0,0));
    auto start_matching = std::chrono::high_resolution_clock::now();

    graspable_points = voxel_matching(voxel_matrix, gripper_mask, matching_settings, gripper_param);

    auto stop_matching = std::chrono::high_resolution_clock::now();
    auto duration_matching = std::chrono::duration_cast<std::chrono::microseconds>(stop_matching - start_matching);
    cout <<"Time for voxel_matching in µs : " << duration_matching.count() << endl;

    // ****** Re-transform ******* //


    std::vector<std::vector<float>> graspable_points_after_retransform;

    // NOTE: Re-transform only to the regression plane frame, NOT to robot-based frame, for better visualization
    // If you want to further retransform to input camera depth optical frame, you have to modify the function

    graspable_points_after_retransform = pcd_re_transform(graspable_points, matching_settings.voxel_size, offset_vector_for_retransform);


    // ****** Visualization ******* //

    // ****** Color Gradient (Criterion I) ******* //
                                                          
    // Turn the graspabiliy score to color gradient. Publish the pointcloud with respect to regression_plane_frame

    sensor_msgs::PointCloud2 marker_of_graspable_points;
    marker_of_graspable_points = visualizeRainbow(graspable_points_after_retransform, matching_settings);

    cout <<"Output cloud with graspability color gradient created"<< endl;

    rainbowPub.publish(marker_of_graspable_points);

    // ****** Curvature Combined (Criterion II) ******* //

    pcl::PointCloud<pcl::PointXYZRGB> peak_cloud;
    sensor_msgs::PointCloud2 results_of_combined_analysis;

    // Searching distance for intersection between convex peaks and graspability maxima
    float distance_threshold = matching_settings.voxel_size;

    // combine Graspability maxima with convex shape analysis. output the intersection of both.
    // Load the PCD file
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../catkin_ws/src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/peak_pcd.pcd", peak_cloud);

    results_of_combined_analysis = combinedAnalysis(graspable_points_after_retransform, peak_cloud, distance_threshold, matching_settings);

    cout <<"graspable points created with combined criterion"<< endl;

    combinedPub.publish(results_of_combined_analysis);

    // ****** Publish the transformed raw point cloud ******* //

    // publish the pointcloud with respect to regrssion_plane_frame
    sensor_msgs::PointCloud2 pointcloud_wrt_regrssion_plane_frame;
    pcl::toROSMsg(new_pcl, pointcloud_wrt_regrssion_plane_frame);
    pointcloud_wrt_regrssion_plane_frame.header.frame_id="regression_plane_frame";
    pointcloud_wrt_regrssion_plane_frame.header.stamp = ros::Time::now();
    transformed_point_pub.publish(pointcloud_wrt_regrssion_plane_frame);

    // ****** Overall time consumption ******* //

    auto stop_overall = std::chrono::high_resolution_clock::now();
    auto duration_overall = std::chrono::duration_cast<std::chrono::microseconds>(stop_overall - start_overall);
    cout <<"Total time in µs : " << duration_overall.count() << endl;


    
    

  }


}

detect_graspable_points::detect_graspable_points()
{

}

/*! ******************************
 ***        Destructor       *****
 *********************************/
detect_graspable_points::~detect_graspable_points()
{
  ;
}

// ****** SECONDARY FUNCTIONS ******* //

std::vector<float> detect_graspable_points::getMinValues(const pcl::PointCloud<pcl::PointXYZ>& pointCloud) {
    std::vector<float> minValues(3);
    std::vector<float> x,y,z;
    pcl::PointXYZ point; //variable for storing point values temporary before adding to pcl
    for (int i = 0; i < pointCloud.size(); ++i)
    {
      point = pointCloud.points[i];
      x.push_back(point.x);
      y.push_back(point.y);
      z.push_back(point.z);
    }

    minValues[0]= *min_element(x.begin(),x.end());
    minValues[1]= *min_element(y.begin(),y.end());
    minValues[2]= *min_element(z.begin(),z.end());

    return minValues;
}



void detect_graspable_points::tf_broadcast(const std::string frame_id)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_depth_optical_frame";
    transformStamped.child_frame_id = frame_id;
    transformStamped.transform.translation.x = 2.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
}

void detect_graspable_points::tf_broadcast_from_pose(const std::string parant_frame_id, const std::string child_frame_id_to, geometry_msgs::Pose relative_pose_between_frame)
{
  // static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parant_frame_id;
  transformStamped.child_frame_id = child_frame_id_to;
  transformStamped.transform.translation.x = relative_pose_between_frame.position.x;
  transformStamped.transform.translation.y = relative_pose_between_frame.position.y;
  transformStamped.transform.translation.z = relative_pose_between_frame.position.z;
  transformStamped.transform.rotation.x = relative_pose_between_frame.orientation.x;
  transformStamped.transform.rotation.y = relative_pose_between_frame.orientation.y;
  transformStamped.transform.rotation.z = relative_pose_between_frame.orientation.z;
  transformStamped.transform.rotation.w = relative_pose_between_frame.orientation.w;
  dynamic_tf.sendTransform(transformStamped);
}

void save3DVectorToFile(const std::vector<std::vector<std::vector<int>>>& vector3D, const std::string& filename) {
    std::ofstream outFile(filename);
    if (!outFile) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    for (int x=0; x<vector3D.size(); x++) 
    {
      for (int y=0; y<vector3D[0].size(); y++)
      {
        for (int z=0; z<vector3D[0][0].size(); z++)
        {
          if (vector3D[x][y][z] != 0){
            outFile << x << "," << y << "," << z << "\n";
          }
        }
      }
    }
    outFile.close();
    std::cout << "3D vector saved to file: " << filename << std::endl;
}


void detect_graspable_points::visualizePoint(const double x, const double y, const double z, const std::string object_name, const std::string frame_id)
{
  visualization_msgs::Marker visualization_point;
	visualization_point.header.frame_id = frame_id;
  visualization_point.ns = object_name;
  visualization_point.id = 0;
  visualization_point.type = visualization_msgs::Marker::SPHERE;
  visualization_point.scale.x = 0.05;	// 0.0 -> warning
  visualization_point.scale.y = 0.05;	// 0.0 -> warning
  visualization_point.scale.z = 0.05;	// 0.0 -> warning
  visualization_point.color.r = 1.0f;
  visualization_point.color.g = 1.0f;
  visualization_point.color.b = 0.0f;
  visualization_point.color.a = 0.8f;
  visualization_point.pose.position.x = x;
  visualization_point.pose.position.y = y;
  visualization_point.pose.position.z = z;
  visualization_point.pose.orientation.x = 0.0;
  visualization_point.pose.orientation.y = 0.0;
  visualization_point.pose.orientation.z = 0.0;
  visualization_point.pose.orientation.w = 1.0;  
  visualization_point.header.stamp = ros::Time::now();
  visualization_point.lifetime = ros::Duration();

  for(int i=0; i<10; i++)
  {
    point_visualization_marker_pub.publish(visualization_point);
    ros::Duration(0.1);
  }
}


void detect_graspable_points::visualizeVector(const Eigen::Vector3f &vector_of_start_point, const Eigen::Vector3f &vector_of_end_point, const std::string object_name, const std::string frame_id)
{
  geometry_msgs::Vector3 arrow;  // config arrow shape
  arrow.x = 0.02;
  arrow.y = 0.04;
  arrow.z = 0.1;
  geometry_msgs::Point initial_point;
  initial_point.x = vector_of_end_point[0];
  initial_point.y = vector_of_end_point[1];
  initial_point.z = vector_of_end_point[2];

  geometry_msgs::Point end_point;
  end_point.x = vector_of_start_point[0] + initial_point.x;
  end_point.y = vector_of_start_point[1] + initial_point.y;
  end_point.z = vector_of_start_point[2] + initial_point.z;

  if(abs(vector_of_start_point.norm()) > 0)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = object_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.points.resize(2);
    marker.points[0] = initial_point;
    marker.points[1] = end_point;
    marker.scale = arrow;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
  }
}

std::vector<int> subtractInteger(const std::vector<int>& vector, int num) {
    std::vector<int> result;
    
    // Perform subtraction for each element
    for (int element : vector) {
        result.push_back(element - num);
    }
    
    return result;
}


// ****** DOWNSAMPLING ******* //

void detect_graspable_points::downsampling(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const std::string frame_id, pcl::PointCloud<pcl::PointXYZ> &filtered_points, const float cube_size)
{
    // VoxelGrid filtering
    // Ref: http://www.pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

    // Container for original & filtered data
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (cube_size, cube_size, cube_size);
    sor.filter (*cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(*cloud_filtered, output);
    output.header.frame_id = frame_id;
    // Publish the data
    downsampled_points_pub.publish(output);

    // get filtered_points as this function's output
    // But this is cause of segmentation fault
    // pcl::fromROSMsg (output, filtered_points);

    // convert from pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ> through sensor_msgs::PointCloud
    sensor_msgs::PointCloud cloud_filtered_by_PC1;
    sensor_msgs::convertPointCloud2ToPointCloud(output, cloud_filtered_by_PC1);
    if(cloud_filtered_by_PC1.points.size() > 0)
    {
      for(auto &filtered_point_by_PC1 : cloud_filtered_by_PC1.points)
      {
        pcl::PointXYZ filtered_point;
        filtered_point.x = filtered_point_by_PC1.x;
        filtered_point.y = filtered_point_by_PC1.y;
        filtered_point.z = filtered_point_by_PC1.z;
        filtered_points.push_back(filtered_point);
      }
    }


    // publish TF for visualization
    const std::string downsampling_frame_ID = "downsampling_frame";
    // to shift positions of rendering point clouds
    tf_broadcast(downsampling_frame_ID);

    // cout << "sec = " << cloud_msg->header.seq << endl;
    //ros::Duration(0.1).sleep();  // If this function is executed in loop function, this sleep might be removed
}

// ****** REGRESSION PLANE ******* //

//Function for least square on pcd
void detect_graspable_points::pcd_least_squares_plane(const pcl::PointCloud<pcl::PointXYZ> raw_pcd, Eigen::Vector4f &centroid,  Eigen::Vector3f &normal_vector_of_plane )
{
   
  Eigen::MatrixXf pcd_matrix = raw_pcd.getMatrixXfMap(3,4,0);//Create a matrix with points as value
  Eigen::MatrixXf deviation_matrix(3,raw_pcd.size());
  Eigen::MatrixXf deviation_matrix_transposed(raw_pcd.size(),3);
  Eigen::Vector3f eigen_values; 
  Eigen::Matrix3f product_deviation_matrix;  // This matrix is used for calculation eigenvector
  Eigen::Vector3f centroid_vector_of_plane3f;

  //compute the centroid
  pcl::compute3DCentroid(raw_pcd,centroid);
  for(int i=0; i< raw_pcd.size()-1; i++)
   {
    deviation_matrix(0,i)=(pcd_matrix(0,i) - centroid(0));
    deviation_matrix(1,i)=(pcd_matrix(1,i) - centroid(1));
    deviation_matrix(2,i)=(pcd_matrix(2,i) - centroid(2)); // substracting the centroid (vector) to each column of the matrix, one column being a point
  }

  deviation_matrix_transposed = deviation_matrix.transpose();
  product_deviation_matrix=deviation_matrix*deviation_matrix_transposed;
  
  //EigenSolver computes the eigen vectors and eigen values but returns them as complex float arrays
  Eigen::EigenSolver<Eigen::MatrixXf> es(product_deviation_matrix);

  //transform complex float vector in float vector
  Eigen::Vector3cf complex_eigen_values = es.eigenvalues();
  eigen_values(0)= real(complex_eigen_values(0));
  eigen_values(1)= real(complex_eigen_values(1));
  eigen_values(2)= real(complex_eigen_values(2));
  
  //sort the smallest eigen values and its corresponding index 
  float a = eigen_values(0);
  int index=0;
  if (eigen_values(1) < a) {
    index = 1;
    a=eigen_values(1);
  }
  if (eigen_values(2) < a) {
    index = 2;
  }

  //choose vector corresponding to the smallest eigen value and convert complex float to float
  Eigen::Vector3cf complex_normal_vector = es.eigenvectors().col(index);
  normal_vector_of_plane(0)= real(complex_normal_vector(0));
  normal_vector_of_plane(1)= real(complex_normal_vector(1));
  normal_vector_of_plane(2)= real(complex_normal_vector(2));

  // cout  << "Normal vector of plane"<<  normal_vector_of_plane<< endl;

  centroid_vector_of_plane3f[0]= centroid[0];
  centroid_vector_of_plane3f[1]= centroid[1];
  centroid_vector_of_plane3f[2]= centroid[2]; 

  visualizeVector(normal_vector_of_plane, centroid_vector_of_plane3f, "normal_vector", "camera_depth_optical_frame");
  
  
}

// ****** TRANSFORMATION ******* //


void detect_graspable_points::pcd_transform ( const pcl::PointCloud<pcl::PointXYZ>  raw_pcd, pcl::PointCloud<pcl::PointXYZ> &transformed_point_cloud, Eigen::Vector4f &centroid_vector_of_plane, Eigen::Matrix3f &rotation_matrix) {


	Eigen::Vector3f normal_vector_of_plane;
	Eigen::Vector3f y_vector_of_plane;
	Eigen::Vector3f x_vector_of_plane;
  Eigen::Vector3f centroid_vector_of_plane3f;
  Eigen::MatrixXf transformed_point_cloud_matrix(3,raw_pcd.size());
  Eigen::MatrixXf raw_pcd_matrix = raw_pcd.getMatrixXfMap(3,4,0);

	pcd_least_squares_plane(raw_pcd ,centroid_vector_of_plane, normal_vector_of_plane);

  //transfering values from 4f vector to 3f vector so .dot computes
  centroid_vector_of_plane3f<< centroid_vector_of_plane[0], centroid_vector_of_plane[1] , centroid_vector_of_plane[2]; 

	float innerproduct_of_centroid_normal_vector = centroid_vector_of_plane3f.dot(normal_vector_of_plane);

   /* 
	if (innerproduct_of_centroid_normal_vector > 0) // changes direction of vector to be from ground to sky if needed
  {
		normal_vector_of_plane = - normal_vector_of_plane;
	}
	 */
	y_vector_of_plane = centroid_vector_of_plane3f.cross(normal_vector_of_plane); // .cross realize cross product
	y_vector_of_plane = y_vector_of_plane/y_vector_of_plane.norm(); //normalize vector
	
  // cout<< "norm of y" << endl << y_vector_of_plane.norm() << endl;

	x_vector_of_plane = normal_vector_of_plane.cross(y_vector_of_plane);

  //assign values to rotation matrix
	rotation_matrix.col(0) = x_vector_of_plane;
	rotation_matrix.col(1) = -y_vector_of_plane; // if no minus sign flipp problem occurs on y axis
	rotation_matrix.col(2) = normal_vector_of_plane;
  // cout << endl << "rotation_matrix" << endl<<  rotation_matrix << endl;


  //operate the frame transformation
  Eigen::Matrix3f rotation_matrix_transposed;
  rotation_matrix_transposed=rotation_matrix.transpose();
  Eigen::MatrixXf raw_pcd_transposed(raw_pcd.size(),3);
  raw_pcd_transposed =raw_pcd_matrix.transpose();
  Eigen::VectorXf one(raw_pcd.size()) ;
  for (int i=0;i<raw_pcd.size();i++)
  {
    one(i)=1;
  }

  transformed_point_cloud_matrix=rotation_matrix_transposed*raw_pcd_matrix - (rotation_matrix_transposed*centroid_vector_of_plane3f*one.transpose());

  //assign matrix values to point that is fed in tranformed pcl
  for (int i = 0; i < raw_pcd.size(); ++i)
  {
    transformed_point_cloud.push_back (pcl::PointXYZ (transformed_point_cloud_matrix(0,i), transformed_point_cloud_matrix(1,i), transformed_point_cloud_matrix(2,i)));
  }
  //cout << "Transformed point cloud" << transformed_point_cloud << endl <<endl;
} 


// ****** INTERPOLATION ******* //

// Diagrams for better understanding:
// /HubRobo/tools/detect_graspable_points/fig_for_understanding/your_image
void detect_graspable_points::pcd_interpolate (const pcl::PointCloud<pcl::PointXYZ>  raw_pcd, pcl::PointCloud<pcl::PointXYZ> &interpolated_point_cloud)
 {
  std::vector<double> x,y,z;
  pcl::PointXYZ point;

  // assign points into vectors for interpolation
  for (int i = 0; i < raw_pcd.size(); ++i)
  {
    point = raw_pcd.points[i];
    x.push_back(point.x);
    y.push_back(point.y);
    z.push_back(point.z);
  }

  // create interpolator
  // .setData add all the known values x,y,z (f(x,y)=z) to the interpolator class
  _2D::LinearDelaunayTriangleInterpolator<double> delaunay_interpolator;
  //_2D::BicubicInterpolator<double> delaunay_interpolator;

  delaunay_interpolator.setData(x,y,z);

  // operation for homogenization and keeping as many points as in input
  double min_y= *min_element(y.begin(),y.end());
  double min_x= *min_element(x.begin(),x.end());
  double max_y= *max_element(y.begin(),y.end());
  double max_x= *max_element(x.begin(),x.end());

  double x_width = max_x-min_x;
  double y_width = max_y-min_y;

  //double grid_size = 1/(round(sqrt(raw_pcd.size()/(x_width*y_width)*(5/3))));
  double grid_size = 1/(round(sqrt(raw_pcd.size() / (x_width * y_width)*(5/3) ) * 10000) / 10000);

  // creates the regular vector x and y for the grid
  std::vector<double> x_grid_vector, y_grid_vector;

  for (double i = min_y; i < max_y; i+=grid_size)
  {
    y_grid_vector.push_back(i);
  }

  for (double i = min_x; i < max_x; i+=grid_size)
  {
    x_grid_vector.push_back(i);
  }
  
  // interpolate the values based on x and y vectors like a grid
  double interp_z;
  for (int i = 0; i < x_grid_vector.size(); ++i) 
  {
    for (int j = 0; j < y_grid_vector.size(); ++j) 
    {
      interp_z= delaunay_interpolator(x_grid_vector[i],y_grid_vector[j]);  // interpolate z at coorinates x,y from given data set
      if (interp_z != 0)          // if the point is interpolated at z=0 then it is not part of original range of data so we don't add it
      {
        interpolated_point_cloud.push_back(pcl::PointXYZ(x_grid_vector[i],y_grid_vector[j],interp_z)); //add the interpolated point
      }
      
    }
  }

}

// ****** VOXELIZATION ******* //

vector<vector<vector<int>>> detect_graspable_points::pcd_voxelize (const pcl::PointCloud<pcl::PointXYZ>  input_pcd, const float cube_size){
  
  //voxel grid creates voxels of said size and then only keeps the centroid of voxels that have points in them
  vector<vector<vector<int>>> voxelized_pcd(0, std::vector<std::vector<int>>(0, std::vector<int>(0,0)));
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;

  std::vector<float> x,y,z;
  pcl::PointXYZ point; //variable for storing point values temporary before adding to pcl
  for (int i = 0; i < input_pcd.size(); ++i)
  {
    point = input_pcd.points[i];
    x.push_back(point.x);
    y.push_back(point.y);
    z.push_back(point.z);
  }

  float min_y= *min_element(y.begin(),y.end());
  float min_x= *min_element(x.begin(),x.end());

  float max_y= *max_element(y.begin(),y.end());
  float max_x= *max_element(x.begin(),x.end());

  float min_z= *min_element(z.begin(),z.end());
  float max_z= *max_element(z.begin(),z.end());

  //size for the 3d array that will receive the voxels informations 
  int xmatrix_size,ymatrix_size,zmatrix_size;

  //size of the matrix is based on the range of data and how many cubes can fit inside
  //ex : from 0 to 9 you can fit 3 cube of size 3
  xmatrix_size= trunc((max_x-min_x)/cube_size)+1;
  ymatrix_size= trunc((max_y-min_y)/cube_size)+1;
  zmatrix_size= trunc((max_z-min_z)/cube_size)+1;


  // change data type for adding it to class
  pcl::PCLPointCloud2 point_cloud;
  pcl::PCLPointCloud2::Ptr point_cloud_2format (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(input_pcd,point_cloud);
  *point_cloud_2format=point_cloud;

  voxel_grid.setInputCloud(point_cloud_2format);
  voxel_grid.setLeafSize(cube_size,cube_size,cube_size);

  // compute a filter to only keep centroid point of voxel
  pcl::PointCloud<pcl::PointXYZ> pcl_after_filtering;
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  voxel_grid.filter(*cloud_filtered);  //fails if the pointcloud is too big
  pcl::fromPCLPointCloud2 (*cloud_filtered, pcl_after_filtering);

  // resize the matrix and fills it with 0 
  voxelized_pcd.resize(xmatrix_size, std::vector<std::vector<int>>(ymatrix_size, std::vector<int>(zmatrix_size,0))); 

  int voxel_index_x,voxel_index_y,voxel_index_z;
  
  for (int i=0; i < pcl_after_filtering.size(); i++)
  {
    // offset the point by the minimum coordinate so minimum is now 0 then divide by resolution to know in which voxel the point is. 
    // trunc it to only keep integer part. refer to : /HubRobo/tools/detect_graspable_points/fig_for_understanding/step_of_voxelize.png

    point=pcl_after_filtering.points[i];
    voxel_index_x= trunc((point.x-min_x)/cube_size + cube_size/4); 
    voxel_index_y= trunc((point.y-min_y)/cube_size + cube_size/4);
    voxel_index_z= trunc((point.z-min_z)/cube_size + cube_size/4);

    voxelized_pcd[voxel_index_x][voxel_index_y][voxel_index_z]=1;
  }



  // You may want to visualize the voxelized array
  /*
  std::string filename = "/home/j02-scare/Documents/Results/vector3D.csv";
  save3DVectorToFile(voxelized_pcd, filename); 
  */

  return voxelized_pcd;


}

//old vox_compare() function, not in use anymore
/* 
std::string detect_graspable_points::vox_compare(const int number_of_points, const vector<vector<vector<int>>> subset_of_voxel_array, const vector<vector<vector<int>>> gripper_mask){

  int number_of_proper_points=0;
  int number_of_obstacle_points = 0;


  //Count the number of solid voxels in the terrain array that lie inside the solid region of the gripper mask.

  for (int x=0; x<subset_of_voxel_array.size(); x++) 
  {
    for (int y=0; y<subset_of_voxel_array[0].size(); y++)
    {
      for (int z=0; z<subset_of_voxel_array[0][0].size(); z++)
      {
        number_of_proper_points += subset_of_voxel_array[x][y][z]*gripper_mask[x][y][z];
      }
    }
  }
  // if numbers are equal then it is graspable

  if (number_of_points != number_of_proper_points) {

    if (number_of_proper_points < ((number_of_points - number_of_proper_points) * penalty_coefficient)){
      return "no";
    } else {
      return "sub";
    }
  }
  
  return "regular";

}
 */

// ****** GRASPABILITY SCORE ******* //

float detect_graspable_points::vox_evaluate(const int number_of_points, const vector<vector<vector<int>>>& subset_of_voxel_array, const vector<vector<vector<int>>>& gripper_mask) {
  int number_of_proper_points = 0;
  const int x_size = subset_of_voxel_array.size();
  const int y_size = subset_of_voxel_array[0].size();
  const int z_size = subset_of_voxel_array[0][0].size();

  for (int x = 0; x < x_size; x++) {
    for (int y = 0; y < y_size; y++) {
      for (int z = 0; z < z_size; z++) {
        number_of_proper_points += subset_of_voxel_array[x][y][z] * gripper_mask[x][y][z];
      }
    }
  }

  float graspability = (static_cast<float>(number_of_proper_points) / number_of_points) * 100.0;
  return graspability;
}


// ****** VOXEL CLIP ******* //

vector<vector<vector<int>>> detect_graspable_points::vox_clip(const int x, const int y, vector<vector<vector<int>>> search_voxel_array) 
{

//crop in x direction
  for (int i=search_voxel_array.size()-x; i<search_voxel_array.size();++i)
  {
    for (int j=0; j<search_voxel_array[0].size();++j)
    {
      for (int k=0; k<search_voxel_array[0][0].size();++k)
        {
          search_voxel_array[i][j][k]=0;
        }
    }
  }

  for (int i=0; i<x; ++i)
  {
    for (int j=0; j<search_voxel_array[0].size();++j)
    {
      for (int k=0; k<search_voxel_array[0][0].size();++k)
        {
          search_voxel_array[i][j][k]=0;
        }
    }
  }

//crop in y direction

  for (int i = 0 ; i < search_voxel_array.size() ; ++i)
  {
    for (int j = search_voxel_array[0].size()-y ; j<search_voxel_array[0].size();++j)
    {
      for (int k=0; k<search_voxel_array[0][0].size();++k)
        {
          search_voxel_array[i][j][k]=0;
        }
    }
  }

  for (int i = 0 ; i < search_voxel_array.size() ; ++i)
  {
    for (int j = 0; j<y; ++j)
    {
      for (int k=0; k<search_voxel_array[0][0].size();++k)
        {
          search_voxel_array[i][j][k]=0;
        }
    }
  }
  return search_voxel_array;
}


// ****** VOXEL EXTRACT ******* //
/* 
vector<vector<vector<int>>> detect_graspable_points::vox_extract(const vector<vector<vector<int>>>& voxel_array,
                                                        const vector<int>& position_reference_point,
                                                        const vector<int>& size_extracting) {

    int size_x = size_extracting[0];
    int size_y = size_extracting[1];
    int size_z = size_extracting[2];

    int ref_x = position_reference_point[0];
    int ref_y = position_reference_point[1];
    int ref_z = position_reference_point[2];

    vector<vector<vector<int>>> extracted_voxel_array(size_x, vector<vector<int>>(size_y, vector<int>(size_z, 0)));

    for (int i = 0; i < size_x; ++i) {
        for (int j = 0; j < size_y; ++j) {
            for (int k = 0; k < size_z; ++k) {
                extracted_voxel_array[i][j][k] = voxel_array[ref_x + i][ref_y + j][ref_z + k];
            }
        }
    }

    return extracted_voxel_array;
}
 */

vector<vector<vector<int>>> detect_graspable_points::vox_extract(const vector<vector<vector<int>>>& voxel_array,
                                                        const vector<int>& position_reference_point,
                                                        const vector<int>& size_extracting,
                                                        const float palm_diameter,
                                                        const float closing_angle,
                                                        const float voxel_size) {

    int size_x = size_extracting[0];
    int size_y = size_extracting[1];
    int size_z = size_extracting[2];

    int ref_x = position_reference_point[0];
    int ref_y = position_reference_point[1];
    int ref_z = position_reference_point[2];

    float ratio = 1.0f / (voxel_size * 1000.0f);

    //float palm_diameter = gripper_param.palm_diameter * ratio;
    //float margin_of_top_solid_diameter = gripper_param.margin_of_top_solid_diameter * ratio;
    float gripper_mask_top_solid_radius = std::round(palm_diameter * ratio / 2);
    //float gripper_mask_clearance = std::round((size_x - palm_diameter) / 2 * std::tan((90 - 80) * (M_PI / 180.0)));

    vector<vector<vector<int>>> extracted_voxel_array(size_x, vector<vector<int>>(size_y, vector<int>(size_z, 0)));
    vector<vector<vector<int>>> control_volume(size_x, vector<vector<int>>(size_y, vector<int>(size_z, 0)));


    for (int z = 0; z < size_z; ++z) {
      float outer_unreachable_radius = std::round(gripper_mask_top_solid_radius + std::sqrt(2 * (size_z-1)/cos(closing_angle * (M_PI / 180.0)) * (z) - std::pow((z), 2)));
      
        for (int x = 0; x < size_x; ++x) {
            for (int y = 0; y < size_y; ++y) {
              float radius = std::sqrt(std::pow(size_x/2 - x, 2) + std::pow(size_y/2 - y, 2));
                if(radius <= outer_unreachable_radius){
                    
                    extracted_voxel_array[x][y][z] = voxel_array[ref_x + x][ref_y + y][ref_z + z];

                    //control_volume[x][y][z] = 1;


                }
            }
        }
    }

    // Save the mask for visualization (best using python!)
    //std::string filename = "../catkin_ws/src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/ControlVolume.csv";
    //save3DVectorToFile(control_volume, filename); 

    

    return extracted_voxel_array;
}



// ****** GRIPPER MASK ******* //

std::vector<std::vector<std::vector<int>>> detect_graspable_points::creategrippermask(GripperParam gripper_param, float voxel_size, const MatchingSettings& matching_settings) {
    // Create a cuboid shaped gripper mask inspired by the MATLAB function by KEIGO HAJI
    // BUT there are some different aspects. See README
    // Currently working on a cylindric shaped mask, still need time to implement

    // Calculate the ratio of voxel size and 1mm in order to keep the gripper size in real world regardless of voxel size
    float ratio = 1.0f / (voxel_size * 1000.0f);

    // Reduce or magnify the gripper's parameters to fit voxel's dimension. Change demensions from [mm] to [voxels]
    float palm_diameter = gripper_param.palm_diameter * ratio;
    float palm_diameter_of_finger_joints = std::round(gripper_param.palm_diameter_of_finger_joints * ratio);
    float finger_length = std::round(gripper_param.finger_length * ratio);
    float spine_length = std::round(gripper_param.spine_length * ratio);
    float spine_depth = std::round(gripper_param.spine_depth * ratio);
    float opening_spine_radius = std::round(gripper_param.opening_spine_radius * ratio);
    float opening_spine_depth = std::round(gripper_param.opening_spine_depth * ratio);
    float closing_height = std::round(gripper_param.closing_height * ratio);
    float margin_of_top_solid_diameter = gripper_param.margin_of_top_solid_diameter * ratio;
    float inside_margin_of_bottom_void_diameter = std::round(gripper_param.inside_margin_of_bottom_void_diameter * ratio);

    // Set the gripper-mask size
    float gripper_mask_half_size = (palm_diameter_of_finger_joints / 2) + finger_length + spine_length;
    float gripper_mask_size = 2 * gripper_mask_half_size;
    float gripper_mask_height = std::round(closing_height);

    // Calculate the parameters to determine solid area and void area
    float gripper_mask_top_solid_radius = std::round((palm_diameter + margin_of_top_solid_diameter) / 2);
    float gripper_mask_clearance = std::round((gripper_mask_size - palm_diameter) / 2 * std::tan((90 - gripper_param.opening_angle) * (M_PI / 180.0)));
    float gripper_mask_bottom_void_radius = std::round(palm_diameter / 2 + (gripper_mask_height * std::tan((gripper_param.closing_angle) * (M_PI / 180.0))) - inside_margin_of_bottom_void_diameter);

    // Prepare a 3-dimensional array composed of 0
    std::vector<std::vector<std::vector<int>>> gripper_mask(gripper_mask_size, std::vector<std::vector<int>>(gripper_mask_size, std::vector<int>(gripper_mask_height, 0)));

    float grippable_radius = 0;
    //float unreachble_radius = 0;
    float outer_unreachable_radius = 0;
    float inner_unreachable_radius = 0;
    float distance_from_center_of_layer = 0;

    cout << "gripper_mask_clearance: " << gripper_mask_clearance << endl;
    cout << "gripper_mask_bottom_void_radius: " << gripper_mask_bottom_void_radius << endl;
    cout << "gripper_mask_top_solid_radius: " << gripper_mask_top_solid_radius << endl;

    // Make the gripper_mask by setting the elements of 1.
    for (int z_subscript = 1; z_subscript < gripper_mask_height + 1; ++z_subscript) {
        
        // Calculate radius of inner cone and outer solid and void area.
        grippable_radius = std::round(gripper_mask_top_solid_radius + (z_subscript-1)/std::tan((90 - gripper_param.opening_angle) * (M_PI / 180.0)));
        outer_unreachable_radius = std::round(gripper_mask_top_solid_radius + std::sqrt(2 * gripper_mask_height/cos(gripper_param.closing_angle * (M_PI / 180.0)) * (gripper_mask_height-(z_subscript-1)) - std::pow(gripper_mask_height-(z_subscript-1), 2)));
        inner_unreachable_radius = std::round(std::sqrt(2*1.5*gripper_mask_bottom_void_radius*((gripper_mask_height-2.5*gripper_mask_bottom_void_radius)+(z_subscript-1))-std::pow(((gripper_mask_height-2.5*gripper_mask_bottom_void_radius)+(z_subscript-1)),2)));
        //unreachble_radius = gripper_mask_half_size - std::round(gripper_mask_half_size - (opening_spine_radius + spine_depth)) * (z_subscript) / (opening_spine_depth - 1);

        for (int y_subscript = 0; y_subscript < gripper_mask_size; ++y_subscript) {
            for (int x_subscript = 0; x_subscript < gripper_mask_size; ++x_subscript) {
                
                // Caculate the distance from center of layer.
                distance_from_center_of_layer = std::sqrt(std::pow(gripper_mask_half_size - x_subscript, 2) + std::pow(gripper_mask_half_size - y_subscript, 2));
                
                // Judges whether it is a solid(1) region or not.
                if ((z_subscript <= gripper_mask_clearance && distance_from_center_of_layer <= grippable_radius) ||
                    (z_subscript > gripper_mask_clearance && z_subscript <= (gripper_mask_height-gripper_mask_bottom_void_radius)
                    && distance_from_center_of_layer <= outer_unreachable_radius
                    
                    ) ||
                    (z_subscript > (gripper_mask_height-gripper_mask_bottom_void_radius) && z_subscript != gripper_mask_height 
                    //&& distance_from_center_of_layer < gripper_mask_half_size 
                    && distance_from_center_of_layer <= outer_unreachable_radius 
                    && distance_from_center_of_layer > inner_unreachable_radius
                    ) ||
                    //(z_subscript > (gripper_mask_height-gripper_mask_bottom_void_radius) && distance_from_center_of_layer > inner_unreachable_radius)
                    //(z_subscript > gripper_mask_clearance && z_subscript != gripper_mask_height) ||
                    (z_subscript == gripper_mask_height && distance_from_center_of_layer > gripper_mask_bottom_void_radius 
                    //&& distance_from_center_of_layer < gripper_mask_half_size
                    && distance_from_center_of_layer <= outer_unreachable_radius
                    ))
                    {
                    
                    // Set those element as 1.
                    gripper_mask[x_subscript][y_subscript][gripper_mask_height - z_subscript] = 1;
                    }
            }
        }
    }

    //cout << "height of gripper mask: " << gripper_mask[0][0].size() << endl;
    
    // Add auxiliary void layers on top of the mask, so that terrain above the mask (in case of inclines for example) would be taken into account.
    for (int i = 0; i < gripper_mask_size; ++i) {
        for (int j = 0; j < gripper_mask_size; ++j) {
            gripper_mask[i][j].insert(gripper_mask[i][j].end(), matching_settings.extra_sheet, 0);
        }
    }

    // Save the mask for visualization (best using python!)
    std::string filename = "../catkin_ws/src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/GripperMask.csv";
    save3DVectorToFile(gripper_mask, filename); 

    return gripper_mask;
}


// ****** CURVATURE ANALYSIS ******* //

void detect_graspable_points::detectTerrainPeaks(pcl::PointCloud<pcl::PointXYZ> input_cloud, sensor_msgs::PointCloud2 &cloud_msg, const MatchingSettings& matching_settings) {
    
    // This function carries out a convex peak detection using curvature analysis. First, we compute the surface normal vectors
    // of each point in a defined radius "searching_radius_for_normal_and_curvature". Next, we compute the respective principal
    // curvatures k1 and k2. Convex peaks are defined as points with an positive k1, k2 and k1*k2=K (Gaussian curvature).
    // TODO: there is a bug in this function. The loop dies as soon as this function finishes. Thus, as a preliminary solution,
    // we save the pcd as a file and go on processing with curvature detection turned "off".
    
    // Output point cloud for peak visualization
    pcl::PointCloud<pcl::PointXYZRGB> peak_visualization_cloud;

    // Create a normal estimation object
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input_cloud.makeShared());

    // Create a KD-Tree for searching
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Compute normals
    ne.setRadiusSearch(matching_settings.searching_radius_for_normal_and_curvature); // Adjust the radius as needed
    ne.compute(*cloud_normals);

    // Create a curvature estimation object
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> est;
    est.setInputCloud(input_cloud.makeShared());
    est.setInputNormals(cloud_normals);
    est.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures> ());


    // Compute curvature
    est.setRadiusSearch(matching_settings.searching_radius_for_normal_and_curvature); // Adjust the radius as needed
    est.compute(*cloud_curvatures);

    // Detect peaks based on positive curvature values and color them blue
    for (size_t i = 0; i < cloud_curvatures->size(); ++i) {
        float curvature1 = cloud_curvatures->points[i].pc1; // Use the principal curvature 1
        float curvature2 = cloud_curvatures->points[i].pc2; // Use the principal curvature 2

        float normal_z = cloud_normals->points[i].normal_z ; // Take the z value of the principal curvature

        /* 
        curvature1 and curvature2 indicate the maximum and minimum eigenvalues of the principal curvature.
        (curvature1 * curvature2) is called Gaussian curvature and it is >0 if the surface is dome-shaped.
        if it is <0, the surface is hyperbloid shaped. if =0, it is cylinder shaped.
        the normal z value shows in the direction of the center of a curvature's radius, so in order to find
        the convex shaped peaks and avoid concave areas, we require points whose normal z value is positive.
        */
        if ((curvature1 * curvature2) > 0.0005 
            && curvature1 > 0
            && curvature2 > 0 
            //&& normal_z < 0.0
            && input_cloud.points[i].z > matching_settings.delete_lower_targets_threshold
            ) { // Positive curvature indicates a peak
            pcl::PointXYZRGB blue_peak;
            blue_peak.x = input_cloud.points[i].x;
            blue_peak.y = input_cloud.points[i].y;
            blue_peak.z = input_cloud.points[i].z;
            blue_peak.r = 0; // Red component
            blue_peak.g = 0; // Green component
            blue_peak.b = 255; // Blue component
            peak_visualization_cloud.push_back(blue_peak);
        }
    }

    // Publish to ROS
    pcl::toROSMsg(peak_visualization_cloud, cloud_msg);
    cloud_msg.header.frame_id="regression_plane_frame";
    cloud_msg.header.stamp = ros::Time::now();

    cout <<"curvature detected."<< endl;

    // Save as pcd
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB>("../catkin_ws/src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/peak_pcd.pcd", peak_visualization_cloud);
    cout << "Saved " << peak_visualization_cloud.size () << " data points to peak_pcd.pcd." << endl;

    //return cloud_msg;

}

// ****** VOXEL MATCHING ******* //


vector<vector<int>> detect_graspable_points::voxel_matching(vector<vector<vector<int>>>& terrain_matrix, const vector<vector<vector<int>>>& gripper_mask, const MatchingSettings& matching_settings, GripperParam gripper_param)
{

    // ****** preparations ******* //

    // Copy inputted terrain data
    std::vector<std::vector<std::vector<int>>> searching_voxel_array = terrain_matrix;
    
    
    // size_of_voxel_array is a 3 element vector filled with the sizes of terrain_matrix
    std::vector<int> size_of_voxel_array = {static_cast<int>(terrain_matrix.size()), static_cast<int>(terrain_matrix[0].size()), static_cast<int>(terrain_matrix[0][0].size())};

    std::vector<int> size_of_gripper_mask = {static_cast<int>(gripper_mask.size()), static_cast<int>(gripper_mask[0].size()), static_cast<int>(gripper_mask[0][0].size())};
    
    std::vector<int> half_size_of_gripper_mask = {static_cast<int>(size_of_gripper_mask[0] / 2), static_cast<int>(size_of_gripper_mask[1] / 2), static_cast<int>(size_of_gripper_mask[2] / 2)};


    // Save z subscript of solid voxels. first, find indices and values of nonzero elements in the terrain_matrix. then,
    // convert linear indices to subscripts
    int z_max = 0;


    for (int i = 0; i < size_of_voxel_array[0]; ++i) {
        for (int j = 0; j < size_of_voxel_array[1]; ++j) {
            for (int k = 0; k < size_of_voxel_array[2]; ++k) {
                if (terrain_matrix[i][j][k] != 0) {
                    if (k > z_max) {
                      z_max = k;
                    }
                }
            }
        }
    }

    // Now, insert empty voxel layers in z-direction bottom, minus the auxiliary gripper mask layers
    std::vector<int> placeholderZeros(size_of_gripper_mask[2]-matching_settings.extra_sheet, 0);

    for (int i = 0; i < size_of_voxel_array[0]; ++i) {
        for (int j = 0; j < size_of_voxel_array[1]; ++j) {
            terrain_matrix[i][j].insert(terrain_matrix[i][j].begin(), placeholderZeros.begin(), placeholderZeros.end());
        }
    }

    // Crop edges of the searching voxel array
    searching_voxel_array = vox_clip(half_size_of_gripper_mask[0]+1, half_size_of_gripper_mask[1]+1, searching_voxel_array);

    // Find all "ones" (solid voxels) in the search voxel array and change indexes to subscripts of solid voxels
    std::vector<int> size_of_searching_voxel_array = {static_cast<int>(searching_voxel_array.size()), static_cast<int>(searching_voxel_array[0].size()), static_cast<int>(searching_voxel_array[0][0].size())};
    Subscripts subscripts_of_searching_solid_voxels;

    for (int i = 0; i < size_of_searching_voxel_array[0]; ++i) {
        for (int j = 0; j < size_of_searching_voxel_array[1]; ++j) {
            for (int k = 0; k < size_of_searching_voxel_array[2]; ++k) {
                if (searching_voxel_array[i][j][k] != 0) {
                    subscripts_of_searching_solid_voxels.x.push_back(i);
                    subscripts_of_searching_solid_voxels.y.push_back(j);
                    subscripts_of_searching_solid_voxels.z.push_back(k);
                }
            }
        }
    }



    // Correct the positions of searching voxels
    subscripts_of_searching_solid_voxels.x = subtractInteger(subscripts_of_searching_solid_voxels.x, half_size_of_gripper_mask[0]);
    subscripts_of_searching_solid_voxels.y = subtractInteger(subscripts_of_searching_solid_voxels.y, half_size_of_gripper_mask[1]);

    int number_of_solid_voxels_in_searching_voxel_array = subscripts_of_searching_solid_voxels.x.size();

    // Prepare for loop
    std::vector<std::vector<int>> searching_solid_voxels_map(4, std::vector<int>(number_of_solid_voxels_in_searching_voxel_array));

    cout <<"Size of number_of_solid_voxels_in_searching_voxel_array:"<< number_of_solid_voxels_in_searching_voxel_array << endl;
    
    // Great loop
    for (int index_of_voxel_being_compared = 0; index_of_voxel_being_compared < number_of_solid_voxels_in_searching_voxel_array; ++index_of_voxel_being_compared) {
        
        std::vector<std::vector<std::vector<int>>> subset_of_voxel_array;

        // Extract subset in the same size of the gripper mask from the data voxel array
        subset_of_voxel_array = vox_extract(terrain_matrix,
                                            {subscripts_of_searching_solid_voxels.x[index_of_voxel_being_compared],
                                            subscripts_of_searching_solid_voxels.y[index_of_voxel_being_compared],
                                            subscripts_of_searching_solid_voxels.z[index_of_voxel_being_compared]},
                                            size_of_gripper_mask,
                                            gripper_param.palm_diameter,
                                            gripper_param.closing_angle,
                                            matching_settings.voxel_size);

      
        //break;
        // Initialize and count the number of voxels inside subset
        int number_of_matching_voxels = 0;

        for (const auto& row : subset_of_voxel_array) {
            for (const auto& col : row) {
                number_of_matching_voxels += std::accumulate(col.begin(), col.end(), 0);
            }
        }
        
        
      //this is for testing the subset extracting for alternative concept of the gripper mask. Currently working on the implementation of it.
        /* 
        for (int z_subscript=0; z_subscript<subset_of_voxel_array[0][0].size(); z_subscript++)
        {
          outer_unreachable_radius = std::round(gripper_mask_top_solid_radius + std::sqrt(2 * gripper_mask_half_size* z_subscript - std::pow(z_subscript, 2)));
          for (int y_subscript=0; y_subscript<subset_of_voxel_array[0].size(); y_subscript++)
          {
            for (int x_subscript=0; x_subscript<subset_of_voxel_array.size(); x_subscript++)
            {
              distance_from_center_of_layer = std::sqrt(std::pow(x_subscript, 2) + std::pow(y_subscript, 2));
              
              if ((z_subscript < (gripper_mask_height-gripper_mask_clearance-1) && z_subscript >= (gripper_mask_height-gripper_mask_bottom_void_radius) 
                    && distance_from_center_of_layer < gripper_mask_half_size
                    ) ||
                    (z_subscript < (gripper_mask_height-gripper_mask_bottom_void_radius)
                    //&& distance_from_center_of_layer < gripper_mask_half_size 
                    && distance_from_center_of_layer < outer_unreachable_radius 
                    )){
                      number_of_matching_voxels += subset_of_voxel_array[x_subscript][y_subscript][z_subscript];
                    }
            }
          }
        }
        
         */
        // Compare the two arrays subset and gripper mask and check whether they match or not
        // returns the graspability score, which is an indicator for the suitability for grasping at this point
        float graspability = vox_evaluate(number_of_matching_voxels, subset_of_voxel_array, gripper_mask);

        // Fill in the point into the output data set
        searching_solid_voxels_map[0][index_of_voxel_being_compared] = subscripts_of_searching_solid_voxels.x[index_of_voxel_being_compared] + half_size_of_gripper_mask[0];
        searching_solid_voxels_map[1][index_of_voxel_being_compared] = subscripts_of_searching_solid_voxels.y[index_of_voxel_being_compared] + half_size_of_gripper_mask[1];
        searching_solid_voxels_map[2][index_of_voxel_being_compared] = subscripts_of_searching_solid_voxels.z[index_of_voxel_being_compared];
        
        // The forth column of the output data set is the graspability score. It is reduced by a penalty ratio
        // if the number of solid terrain voxels inside the subset is below a thershold (Threshold of Solid Voxels, TSV)
        if (number_of_matching_voxels > matching_settings.threshold) {
            //cout <<"number of matching voxels exceeded the threshold" << endl;
            searching_solid_voxels_map[3][index_of_voxel_being_compared] = graspability;
          }
        // We only penalize those points which has a erroneous high graspability score
        else if (number_of_matching_voxels <= matching_settings.threshold && graspability >= 60) {
            searching_solid_voxels_map[3][index_of_voxel_being_compared] = graspability - (((matching_settings.threshold - number_of_matching_voxels)*1.0)/(matching_settings.threshold*1.0))*100;
            //cout << "Below threshold! Graspability: " << searching_solid_voxels_map[3][index_of_voxel_being_compared] <<endl;
         }
        else {
          searching_solid_voxels_map[3][index_of_voxel_being_compared] = graspability;
        }

        

        //cout << "graspability:" << searching_solid_voxels_map[3][index_of_voxel_being_compared]  << endl;
        //cout << "Number of voxel being compared: " << index_of_voxel_being_compared <<"/"<< number_of_solid_voxels_in_searching_voxel_array <<endl;

    }
    
    // End of the great loop

    std::vector<std::vector<int>> voxel_coordinates_of_graspable_points;

    // Crop the remaining 0 column
    searching_solid_voxels_map.erase(std::remove_if(searching_solid_voxels_map.begin(), searching_solid_voxels_map.end(),
                                           [](const std::vector<int>& matching) { return std::all_of(matching.begin(), matching.end(), [](int value) { return value == 0; }); }),
                            searching_solid_voxels_map.end());
    

    //Correct the position of the voxel array of the terrain matrix
    //because we make the voxel array in the reverse direction in pcd_voxelize.m
    int max_z_position = z_max;
  
    for (auto& matching : searching_solid_voxels_map) {
        matching[2] = -matching[2] + max_z_position;
    }

    voxel_coordinates_of_graspable_points = searching_solid_voxels_map;
    
    // Store as file if needed
    //std::string filename = "/home/j02-scare/Documents/Results/Voxel_coordinates_of_graspable_points.csv";
    //save2DVectorToFile(voxel_coordinates_of_graspable_points, filename);
    return voxel_coordinates_of_graspable_points;

}


// ****** RE-TRANSFORMATION ******* //

std::vector<std::vector<float>> detect_graspable_points::pcd_re_transform(std::vector<std::vector<int>> voxel_coordinates_of_graspable_points, float voxel_size, std::vector<float> offset_vector) {
                                                                          // Eigen::Matrix3f rotation_matrix, 
                                                                          // Eigen::Vector4f centroid_vector_of_plane, 
                                                                          // std::vector<float> offset_vector
                                                                          
    
    //pcd_re_transform returns the coordinates of the voxel array to the original input coordinate system.

    // Initialize output array.
    std::vector<std::vector<float>> graspable_points(0, vector<float>(0,0));


    if (voxel_coordinates_of_graspable_points.empty()) {
        return graspable_points;
    }

    // Resize output array.
    graspable_points.resize(4, vector<float>(voxel_coordinates_of_graspable_points[0].size(), 0.0f));

    // Re-transformation using the parameters in the voxelization step
    for (int i = 0; i < voxel_coordinates_of_graspable_points[0].size(); ++i) {
         
        graspable_points[0][i] = (voxel_coordinates_of_graspable_points[0][i]*1.0 - voxel_size/4) * voxel_size + offset_vector[0];
        graspable_points[1][i] = (voxel_coordinates_of_graspable_points[1][i]*1.0 - voxel_size/4) * voxel_size + offset_vector[1];
        graspable_points[2][i] = (voxel_coordinates_of_graspable_points[2][i]*1.0 - voxel_size/4) * voxel_size + offset_vector[2];
        graspable_points[3][i] = voxel_coordinates_of_graspable_points[3][i];

    }
    // Store if needed
    /* 
    std::string filename = "/home/jitao/Documents/Results/Voxel_coordinates_of_graspable_points.csv";
    save_graspablepoints(graspable_points, filename);
    */
    return graspable_points;


}

// ****** VISUALIZATION (COLOR GRADIENT) ******* //

sensor_msgs::PointCloud2 detect_graspable_points::visualizeRainbow(std::vector<std::vector<float>> array, const MatchingSettings& matching_settings) {

    // Create a marker message

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;

    for (int n=0; n<array[0].size(); n++) {
            pcl::PointXYZRGB point;

            point.x = array[0][n];
            point.y = array[1][n];
            point.z = array[2][n];

            // Convert the fourth value (0-100) to an RGB color
            point.b = 0; // Set blue to 0
            int color_value = static_cast<int>(array[3][n]);

            //5 different color from bright green to red for the scale of graspability
            if(color_value >= 90) {
              point.r = 8;
              point.g = 144;
            }
            else if(color_value >= 80 && color_value < 90) {
              point.r = 99;
              point.g = 255;
            }
            else if(color_value >= 70 && color_value < 80) {
              point.r = 214;
              point.g = 255;
            }
            else if(color_value >= 60 && color_value < 70) {
              point.r = 255;
              point.g = 255;
            }
            else if(color_value >= 50 && color_value < 60) {
              point.r = 255;
              point.g = 193;
            }
            
            else if(color_value >= 40 && color_value < 50) {
              point.r = 255;
              point.g = 154;
            }
            // More colors for score below 50 if needed
            /* 
            else if(color_value >= 30 && color_value < 45) {
              point.r = 255;
              point.g = 116;
            }
            else if(color_value >= 15 && color_value < 30) {
              point.r = 255;
              point.g = 77;
            }
             */
            // Non-graspable is marked as red
            else {
              point.r = 255;
              point.g = 0;
              //grade_5 = grade_5 + 1;
            }

            // if lower threshold is set, targets below will be set to white
            if(matching_settings.delete_lower_targets == "on" && point.z < matching_settings.delete_lower_targets_threshold) {
              point.r = 255;
              point.g = 255;
              point.b = 255;
            }


            

            pcl_cloud.push_back(point);
        }

    // Create ROS Message for publishing
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_msg.header.frame_id="regression_plane_frame";
    cloud_msg.header.stamp = ros::Time::now();

    return cloud_msg;
    
}


// ****** VISUALIZATION (INTERSECTION CONVEXITY & GRASPABILITY) ******* //

sensor_msgs::PointCloud2 detect_graspable_points::combinedAnalysis(
    const std::vector<std::vector<float>> array, 
    const pcl::PointCloud<pcl::PointXYZRGB> cloud2,
    float distance_threshold,
    const MatchingSettings& matching_settings)
{
    // this function combines the results of the curvature analysis (peaks) and the graspable points and returns a single point cloud
    // with points which belong to both categories

    // Initialize the variables
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud1;
    pcl::PointCloud<pcl::PointXYZRGB> close_points;


    // Take only those points with a higher graspability score of a certain value and if they are above the z-threshold
    // we give them blue color
    for (int n=0; n<array[0].size(); n++) {
            pcl::PointXYZRGB point;

            if (array[3][n] >= matching_settings.graspability_threshold && array[2][n] > matching_settings.delete_lower_targets_threshold){
                point.x = array[0][n];
                point.y = array[1][n];
                point.z = array[2][n];
                point.r = 128; // Set blue to 0
                point.g = 0; // Set blue to 0
                point.b = 128; // Set blue to 0
                cloud1.push_back(point);
            }

            
        }

    // return the intersection of high graspability score points and convex peaks.
    // if both points are closer than a distance_threshold, we take it.
    for (const pcl::PointXYZRGB point1 : cloud1.points) {
        for (const pcl::PointXYZRGB point2 : cloud2.points) {
            float distance = pcl::euclideanDistance(point1, point2);

            if (distance < distance_threshold) {
                close_points.push_back(point1);
                close_points.push_back(point2);
            }
        }
    }


    // Publish to ROS
    cout << "published " << close_points.size () << " data points to ROS" << endl;

    pcl::toROSMsg(close_points, cloud_msg);
    cloud_msg.header.frame_id="regression_plane_frame";
    cloud_msg.header.stamp = ros::Time::now();

    return cloud_msg;
}



