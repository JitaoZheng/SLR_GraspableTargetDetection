cmake_minimum_required(VERSION 3.5)
project(coppelia_interface)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs message_filters)
add_executable(conversion src/conversion.cpp)
ament_target_dependencies(conversion rclcpp sensor_msgs image_geometry message_filters std_msgs cv_bridge image_transport pcl_ros pcl_conversions PCL)
add_executable(icp src/ICP.cpp)
ament_target_dependencies(icp rclcpp sensor_msgs image_geometry geometry_msgs message_filters std_msgs cv_bridge image_transport pcl_ros pcl_conversions PCL tf2 tf2_ros tf2_geometry_msgs)
#add_executable(concatenate src/concatenate.cpp)
#ament_target_dependencies(conversion rclcpp sensor_msgs image_transport pcl_ros)


set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/)

target_include_directories(conversion PUBLIC ${INCLUDE_DIR})
target_include_directories(icp PUBLIC ${INCLUDE_DIR})


install(TARGETS
  talker
  listener
  conversion
  icp
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
