#pragma once

#include <omp.h>
#include <vector>
#include <Eigen/Core>
#include <pcl_ros/point_cloud.h>

using PointT = pcl::PointXYZINormal;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
using PointCloudConstPtr = pcl::PointCloud<PointT>::ConstPtr;

template <typename T>
using MatricesVectorT = std::vector<Eigen::Matrix<T, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 3>>>;

template <typename T>
Eigen::Matrix<T, 3, 3> PCLTwoPlaneVectorsFromNormal(const PointT& normal) {
  Eigen::Matrix<T, 3, 1> vec1, vec2;
  Eigen::Matrix<T, 3, 1> normal_eig(normal._PointXYZINormal::normal_x, normal._PointXYZINormal::normal_y,
                                    normal._PointXYZINormal::normal_z);
  normal_eig.normalize();
  if (std::abs(normal._PointXYZINormal::normal_z) < 0.0000001 || std::abs(normal_eig[2]) < 0.0000001) {
    normal_eig[2] = 0.0000001;
  }
  vec1[0] = 1.0;
  vec1[1] = 0.0;
  vec1[2] = -normal_eig[0] / normal_eig[2];
  vec1.normalize();
  vec2 = normal_eig.cross(vec1);
  vec2.normalize();

  return 0.001 * normal_eig * normal_eig.transpose() + vec1 * vec1.transpose() + vec2 * vec2.transpose();
}

template <typename T>
void CalculateCovarianceFromNormals(const PointCloudConstPtr& point_cloud, MatricesVectorT<T>& cloud_covariances,
                                    int k_num_threads = 1) {
  cloud_covariances.resize(point_cloud->size());
  omp_set_num_threads(k_num_threads);
#pragma omp parallel for schedule(dynamic, 1)
  for (int i = 0; i < point_cloud->points.size(); ++i) {
    cloud_covariances[i] = PCLTwoPlaneVectorsFromNormal<T>(point_cloud->points[i]);
  }
}
