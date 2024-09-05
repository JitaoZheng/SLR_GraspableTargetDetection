cmake_minimum_required(VERSION 3.5)
project(detect_graspable_points)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
#find_package(OpenMP REQUIRED)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/)

include_directories("/usr/include/eigen3")
include_directories("/home/j02-scare2/libInterpolate/src/libInterpolate")

# Add the executable
add_executable(${PROJECT_NAME}_main src/${PROJECT_NAME}_main.cpp
				     src/${PROJECT_NAME}.cpp)



add_executable(publish_pointcloud2 src/publish_pointcloud2.cpp)

ament_target_dependencies(detect_graspable_points_main
                          rclcpp 
                          sensor_msgs 
                          pcl_conversions 
                          pcl_ros 
                          tf2_ros 
                          visualization_msgs 
                          geometry_msgs 
                          PCL)

ament_target_dependencies(publish_pointcloud2
                          rclcpp 
                          sensor_msgs 
                          pcl_conversions 
                          pcl_ros
                          PCL)

            
target_include_directories(${PROJECT_NAME}_main PUBLIC ${INCLUDE_DIR})

# Link against the libraries
target_link_libraries(detect_graspable_points_main ${PCL_LIBRARIES})
target_link_libraries(publish_pointcloud2 ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



#if(OpenMP_CXX_FOUND)
#    target_link_libraries(detect_graspable_points_main PUBLIC OpenMP::OpenMP_CXX)
#    target_compile_options(detect_graspable_points_main PUBLIC ${OpenMP_CXX_FLAGS})
#endif()


# Install the executable
install(TARGETS detect_graspable_points_main publish_pointcloud2
  DESTINATION lib/${PROJECT_NAME})







ament_package()

