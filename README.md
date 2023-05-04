# icp_visualization
Visualize each iteration of ICP-like algorithms in RViz, including residuals, normals, correspondences.

## Dependencies
- Ubuntu 20
- ROS (Follow ROS official website to install)
- CMake, PCL, Eigen, yaml-cpp, OpenMP (we use their default version in Ubuntu 20)
```
sudo apt install cmake libpcl-dev libeigen3-dev libyaml-cpp-dev libomp-dev
```
- Install our customized RViz plugins: clone the following packages into ROS workspace and install.
```
cd ~/catkin_ws/src
git clone https://github.com/UCR-Robotics/correspondence_rviz_plugin
git clone https://github.com/UCR-Robotics/pointcloud2_normal_rviz_plugin
cd ..
catkin_make install
```

## Installation

Place this package under a ROS workspace, and `catkin_make`.

## Usage

1. Open RViz

```
roslaunch icp_visualization rviz.launch
```

2. Run ICP registration program

```
roslaunch icp_visualization icp.launch
```
