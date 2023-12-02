# SRL_GraspableTargetDetection
Create graspable target detection for legged robot

## Authors and Maintainers

*   Jitao Zheng (jitao.zheng@tum.de)

*   Taku Okawara (taku.okawara.t3@dc.tohoku.ac.jp)

*   Kentaro Uno (unoken@astro.mech.tohoku.ac.jp)

## System Requirements

The code was tested on: 
*   ROS Noetic
*   Ubuntu 20.04
*   gcc version 9.4.0

You will require the following packages and libraries:
*   Point Cloud Library (PCL)
*   LibInterpolate
*   Eigen3

## Quick Start


Open a new terminal window.

Type:

```
cd ~/catkin_ws/src
```
Clone the repository:
```
git clone <address>
```

Then we can build it:

```
cd ..

catkin_make
```
If you don't have yet installed Point Cloud Library (PCL), you can install it by typing:

```
sudo apt install libpcl-dev
```

We will first test the algorithm on some examples. You will need at least three terminals.

*Terminal 1*


Roscore.
```
cd catkin_ws
source /opt/ros/noetic/setup.bash
roscore
```
*Terminal 2*

Publish stored example point cloud.
```
cd catkin_ws
source ~/catkin_ws/devel/setup.bash
rosrun detect_graspable_points publish_pointcloud2
```
Now, a point cloud in *.pcd* format will be published once per second under the topic `merged_pcd` in the `regression_plane_frame` coordinate frame.

*Terminal 3*

Open RVIZ.
```
cd catkin_ws
source ~/catkin_ws/devel/setup.bash
roslaunch detect_graspable_points rviz.launch
```
You can freely choose which topic you want to visualize, whether that is the raw point cloud with the graspable points (light green spheres) or the color gradient of Graspability Score.

*Terminal 4*

Launch the graspable target detection.
```
cd catkin_ws
source ~/catkin_ws/devel/setup.bash
roslaunch detect_graspable_points detect_graspable_points.launch
```
Now you should see the point cloud showing up in RVIZ. 

<img src="Images/HubRobo_leaning_bouldering_hold_with_legend.png" alt="drawing" width="600"/>



