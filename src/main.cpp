// Visualization for ICP-like algorithms
// Hanzhe Teng, May 2023

#include "icp_visualization/icp_visualization.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "icp_visualization");
  ros::NodeHandle nh("~");
  icp_vis::ICPVisualization icp_visualization(nh);
  ros::spin();
  ROS_INFO("ICPVisualization terminated");
  return 0;
}
