// Visualization for ICP-like algorithms
// Hanzhe Teng, May 2023

#pragma once

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>
#include <yaml-cpp/yaml.h>

#include <correspondence_rviz_plugin/PointCloudCorrespondence.h>
#include "fast_gicp/fast_gicp.hpp"
#include "locus_gicp/multithreaded_gicp.h"

namespace Eigen {
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, -1> Matrix6Xd;
}  // namespace Eigen

namespace icp_vis {

class ICPVisualization {
 public:
  using PointT = pcl::PointXYZINormal;
  using NormalT = pcl::PointXYZINormal;
  using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

  ICPVisualization(ros::NodeHandle& nh);
  ~ICPVisualization() {}

 protected:
  void DownSampleVoxelGrids(const PointCloudPtr& cloud);
  void EstimateNormals(const PointCloudPtr& cloud);
  void CopyPointCloud(const PointCloudPtr& cloud_in, const std::vector<int>& indices, PointCloudPtr& cloud_out);

  void PublishVisualization(const PointCloudPtr& source, const PointCloudPtr& target, 
                            std::vector<std::vector<double>>& errors_per_iter,
                            std::vector<std::vector<int>>& correspondences_per_iter,
                            std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& transformation_per_iter);

  Eigen::Matrix4d Point2PointICP(const PointCloudPtr& source, const PointCloudPtr& target);
  Eigen::Matrix4d Point2PlaneICP(const PointCloudPtr& source, const PointCloudPtr& target);
  Eigen::Matrix4d GaussNewton(const PointCloudPtr& source, const PointCloudPtr& target, Eigen::VectorXd& Residual);
  Eigen::Matrix4d TransformVector6dToMatrix4d(const Eigen::Vector6d& input);
  Eigen::Matrix4d FastGICP(const PointCloudPtr& source, const PointCloudPtr& target);
  Eigen::Matrix4d LocusGICP(const PointCloudPtr& source, const PointCloudPtr& target);

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_source_cloud_;
  ros::Publisher pub_target_cloud_;
  ros::Publisher pub_correspondence_;
  pcl::search::KdTree<PointT>::Ptr kdtree_;
  pcl::PointCloud<PointT>::Ptr source_cloud_;
  pcl::PointCloud<PointT>::Ptr target_cloud_;
  Eigen::Matrix4d transformation_;
  YAML::Node params_;
};

}  // namespace icp_vis
