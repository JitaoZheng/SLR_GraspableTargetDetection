# detect_graspable_points

Author(s) and maintainer(s):

* Jitao Zheng (jitao.zheng@tum.de)
  
* Ringeval-Meusnier Antonin (ringeval-meusnier.antonin.camille.charles.s3@dc.tohoku.ac.jp)

* Taku Okawara (taku.okawara.t3 at dc.tohoku.ac.jp)

* Kentaro Uno (unoken at astro.mech.tohoku.ac.jp)

## Summary

* This is the package for detecting graspable points from a 3d terrain map by sensor_msgs::PointCloud2.


## Build and Usage

### Environment
This code is tested on the following environment.
* ubuntu 18.04, ROS melodic
* ubuntu 20.04, ROS noetic

### Adding dependency

* Libinterpolate : This library provides classes to perform various types of function interpolation (linear, spline, etc). It can perform interpolation from 1D (y=f(x)) or 2D (z=f(x,y)) set of data.

```

Installation of libinterp:

1)Open a new terminal
    $ git clone https://github.com/CD3/libInterpolate
    $ cd libInterpolate
    $ mkdir build
    $ cd build
    $ make
    $ sudo make install

2) If your environment is ubuntu 20.04 and ROS noetic, please skip 2) and 3). 
   Change version to previous commit to avoid error caused by eigen version and cmake version: 16 Jan 2020 88d49df7

3) Create these new files and copy/paste code from github library page 
    libInterpolate/src/libInterpolate/Interpolators/_2D/LinearDelaunayTriangleInterpolator.hpp
    libInterpolate/src/libInterpolate/Interpolators/_2D/DelaunayTriangulationInterpolatorBase.hpp
    libInterpolate/src/libInterpolate/Utils/Meshing/delaunator-cpp.hpp
    
    modify already existing file by copy and paste from git page:
    /libInterpolate/src/libInterpolate/Interpolators/_2D/InterpolatorBase.hpp 

4) Replace path of new library at line 25, this package's cmakelists.txt : include_directories("/home/YOUR_USERNSME/libInterpolate/src/libInterpolate")
```

### Launching example

```
$ catkin build detect_graspable_points
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch detect_graspable_points example.launch
```



## Reference

Matlab code :
climblab/lib/target_detection/pcd_least_squares_plane
climblab/lib/target_detection/pcd_transform
climblab/lib/target_detection/pcd_interpolate
climblab/lib/target_detection/pcd_voxelize
climblab/lib/target_detection/voxel_matching
climblab/lib/target_detection/vox_clip
climblab/lib/target_detection/vox_extract
climblab/lib/target_detection/vox_compare


Libraries :
libinterpolate : https://github.com/CD3/libInterpolate

Document:
PhD thesis by Kentaro Uno : Autonomus Limbed Climbing Robot for Challenging Terrain Exploration
