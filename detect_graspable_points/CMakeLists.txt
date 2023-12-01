cmake_minimum_required(VERSION 3.0.2)
project(detect_graspable_points)

### adding compile option -std=c++11 by uncommenting the following settings 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++14 -O3 -mtune=native -march=native -mfpmath=both)

# find_package(PCL 1.7 REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  pcl_ros
  sensor_msgs
  pcl_conversions
  visualization_msgs

)
include_directories("/usr/include/eigen3")
include_directories("/home/j02-scare2/libInterpolate/src/libInterpolate")

# find_package(Eigen3 REQUIRED)
# include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs tf pcl_ros sensor_msgs pcl_conversions visualization_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

### Building to add a shared library
# adding the C++ shared library from the specified source
add_library(${PROJECT_NAME}
	src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(${PROJECT_NAME}_main src/${PROJECT_NAME}_main.cpp
				     src/${PROJECT_NAME}/${PROJECT_NAME}.cpp)

# add_executable(ps3_${PROJECT_NAME}_test src/ps3_${PROJECT_NAME}/ps3_${PROJECT_NAME}.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_main
   ${catkin_LIBRARIES}
  #  ${PCL_LIBRARIES}
 )
 
add_executable(publish_pointcloud2 src/publish_pointcloud2.cpp)


target_link_libraries(publish_pointcloud2 ${catkin_LIBRARIES})

