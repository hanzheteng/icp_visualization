// Visualization for ICP-like algorithms
// Hanzhe Teng, May 2023

#include <correspondence_rviz_plugin/PointCloudCorrespondence.h>
#include "icp_visualization/icp_visualization.h"

namespace icp_vis {

ICPVisualization::ICPVisualization(ros::NodeHandle& nh)
    : nh_(nh),
      source_cloud_(new pcl::PointCloud<PointT>),
      target_cloud_(new pcl::PointCloud<PointT>),
      transformation_(Eigen::Matrix4d::Identity()),
      kdtree_(new pcl::search::KdTree<PointT>) {
  // get params
  std::string param_filename, src_filename, tar_filename;
  nh_.getParam("/param", param_filename);
  nh_.getParam("/source_cloud", src_filename);
  nh_.getParam("/target_cloud", tar_filename);

  // initialize utilities
  params_ = YAML::LoadFile(param_filename);
  pub_source_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/source_cloud", 5);
  pub_target_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/target_cloud", 5);
  pub_correspondence_ = nh_.advertise<correspondence_rviz_plugin::PointCloudCorrespondence>("/correspondence", 5);

  // load point clouds
  if (pcl::io::loadPCDFile(src_filename, *source_cloud_) == 0)
    std::cout << "[source] Loaded " << source_cloud_->size() << " points\n";
  if (pcl::io::loadPCDFile(tar_filename, *target_cloud_) == 0)
    std::cout << "[target] Loaded " << target_cloud_->size() << " points\n";

  // pre-processing if needed
  if (params_["downsample"].as<bool>()) {
    DownSampleVoxelGrids(source_cloud_);
    DownSampleVoxelGrids(target_cloud_);
  }
  if (params_["estimate_normal"].as<bool>()) {
    EstimateNormals(source_cloud_);
    EstimateNormals(target_cloud_);
  }

  // registration
  std::string icp_method = params_["icp_method"].as<std::string>();
  if (icp_method == "point2point")
    transformation_ = Point2PointICP(source_cloud_, target_cloud_);
  else if (icp_method == "point2plane")
    transformation_ = Point2PlaneICP(source_cloud_, target_cloud_);
  else
    PCL_ERROR("no registration method is selected");
  std::cout << "Estimated transformation " << std::endl << transformation_ << std::endl;
}

void ICPVisualization::DownSampleVoxelGrids(const PointCloudPtr& cloud) {
  double resolution = params_["downsample_resolution"].as<double>();
  pcl::VoxelGrid<PointT> sor;
  sor.setLeafSize(resolution, resolution, resolution);
  sor.setInputCloud(cloud);
  sor.filter(*cloud);
  std::cout << "Downsampled to " << cloud->size() << " points\n";
}

void ICPVisualization::EstimateNormals(const PointCloudPtr& cloud) {
  pcl::NormalEstimation<PointT, NormalT> ne;
  ne.setSearchMethod(kdtree_);
  ne.setKSearch(params_["normal_est_neighbors"].as<int>());
  ne.setInputCloud(cloud);
  ne.compute(*cloud);
  std::cout << "Computed " << cloud->size() << " normals\n";
}

void ICPVisualization::CopyPointCloud(const PointCloudPtr& cloud_in, const std::vector<int>& indices,
                                      PointCloudPtr& cloud_out) {
  // allocate enough space and copy the basics
  cloud_out->points.resize(indices.size());
  cloud_out->header = cloud_in->header;
  cloud_out->width = static_cast<uint32_t>(indices.size());
  cloud_out->height = 1;
  cloud_out->is_dense = cloud_in->is_dense;
  cloud_out->sensor_orientation_ = cloud_in->sensor_orientation_;
  cloud_out->sensor_origin_ = cloud_in->sensor_origin_;

  // iterate over each point
  for (size_t i = 0; i < indices.size(); ++i) {
    cloud_out->points[i] = cloud_in->points[indices[i]];
  }
}

// Standard point-to-point ICP registration using SVD (closed-form solution; no need to run optimization)
// exactly the same behavior and performance as in PCL
// refer to pcl/registration/impl/icp.hpp and transformation_estimation_svd.hpp
Eigen::Matrix4d ICPVisualization::Point2PointICP(const PointCloudPtr& source, const PointCloudPtr& target) {
  // initialization
  int max_iterations = params_["icp_max_iterations"].as<int>();
  double max_distance = params_["icp_max_corres_dist"].as<double>();
  double translation_epsilon = params_["icp_translation_epsilon"].as<double>();
  double rotation_epsilon = 1.0 - params_["icp_rotation_epsilon"].as<double>();
  int cloud_size = static_cast<int>(source->size());
  Eigen::Matrix4d final_transformation = Eigen::Matrix4d::Identity();
  pcl::PointCloud<PointT>::Ptr source_trans(new pcl::PointCloud<PointT>);
  ros::Rate rate(1);

  // build K-d tree for target cloud
  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
  kdtree->setInputCloud(target);
  std::vector<int> indices(1);    // for nearestKSearch
  std::vector<float> sq_dist(1);  // for nearestKSearch

  // prepare visualization data
  correspondence_rviz_plugin::PointCloudCorrespondence correspondence_msg;
  sensor_msgs::PointCloud2 cloud_source;
  sensor_msgs::PointCloud2 cloud_target;
  pcl::toROSMsg(*target, cloud_target);
  cloud_target.header.frame_id = "map";
  correspondence_msg.cloud_target = cloud_target;
  correspondence_msg.header.frame_id = "map";

  // repeat until convergence
  for (int t = 0; t < max_iterations; ++t) {
    // transform source using estimated transformation
    pcl::transformPointCloud<PointT>(*source, *source_trans, final_transformation);

    // find correspondences in target
    std::vector<int> indices_src;
    std::vector<int> indices_tgt;
    for (int i = 0; i < cloud_size; ++i) {
      kdtree->nearestKSearch(source_trans->points[i], 1, indices, sq_dist);
      if (sq_dist[0] > max_distance * max_distance)  // skip if too far
        continue;
      indices_src.push_back(i);
      indices_tgt.push_back(indices[0]);
    }

    // convert to Eigen format
    int indices_size = static_cast<int>(indices_src.size());
    Eigen::Matrix3Xd cloud_src(3, indices_size);
    Eigen::Matrix3Xd cloud_tgt(3, indices_size);
    for (int i = 0; i < indices_size; ++i) {
      cloud_src(0, i) = source_trans->points[indices_src[i]].x;
      cloud_src(1, i) = source_trans->points[indices_src[i]].y;
      cloud_src(2, i) = source_trans->points[indices_src[i]].z;
      cloud_tgt(0, i) = target->points[indices_tgt[i]].x;
      cloud_tgt(1, i) = target->points[indices_tgt[i]].y;
      cloud_tgt(2, i) = target->points[indices_tgt[i]].z;
    }

    // skip a few steps (such as sanity checks) here for simplicity

    // solve using Umeyama's algorithm (SVD)
    Eigen::Matrix4d transformation = Eigen::umeyama<Eigen::Matrix3Xd, Eigen::Matrix3Xd>(cloud_src, cloud_tgt, false);
    final_transformation = transformation * final_transformation;
    std::cout << "iter = " << t << "; cloud size = " << cloud_size << "; indices_size = " << indices_size << std::endl;

    // prepare visualization data
    for (int i = 0; i < cloud_size; ++i) {
      source_trans->points[i].intensity = 0;
    }
    correspondence_msg.index_source = indices_src;
    correspondence_msg.index_target = indices_tgt;
    pcl::toROSMsg(*source_trans, cloud_source);
    cloud_source.header.frame_id = "map";
    correspondence_msg.cloud_source = cloud_source;
    pub_correspondence_.publish(correspondence_msg);
    pub_source_cloud_.publish(cloud_source);
    pub_target_cloud_.publish(cloud_target);

    // check convergence
    double cos_angle = 0.5 * (transformation.coeff(0, 0) + transformation.coeff(1, 1) + transformation.coeff(2, 2) - 1);
    double translation_sqr = transformation.coeff(0, 3) * transformation.coeff(0, 3) +
                             transformation.coeff(1, 3) * transformation.coeff(1, 3) +
                             transformation.coeff(2, 3) * transformation.coeff(2, 3);
    if (cos_angle >= rotation_epsilon && translation_sqr <= translation_epsilon) {
      std::cout << "converged!" << std::endl;
      break;
    }
    if (!ros::ok()) break;
    rate.sleep();
  }

  return final_transformation;
}

// Point-to-plane ICP registration by Gauss-Newton optimization
Eigen::Matrix4d ICPVisualization::Point2PlaneICP(const PointCloudPtr& source, const PointCloudPtr& target) {
  // initialization
  int max_iterations = params_["icp_max_iterations"].as<int>();
  double max_distance = params_["icp_max_corres_dist"].as<double>();
  double translation_epsilon = params_["icp_translation_epsilon"].as<double>();
  double rotation_epsilon = 1.0 - params_["icp_rotation_epsilon"].as<double>();
  int cloud_size = static_cast<int>(source->size());
  Eigen::Matrix4d final_transformation = Eigen::Matrix4d::Identity();
  pcl::PointCloud<PointT>::Ptr source_trans(new pcl::PointCloud<PointT>);
  ros::Rate rate(1);

  // build K-d tree for target cloud
  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
  kdtree->setInputCloud(target);
  std::vector<int> indices(1);    // for nearestKSearch
  std::vector<float> sq_dist(1);  // for nearestKSearch

  // prepare visualization data
  correspondence_rviz_plugin::PointCloudCorrespondence correspondence_msg;
  sensor_msgs::PointCloud2 cloud_source;
  sensor_msgs::PointCloud2 cloud_target;
  pcl::toROSMsg(*target, cloud_target);
  cloud_target.header.frame_id = "map";
  correspondence_msg.cloud_target = cloud_target;
  correspondence_msg.header.frame_id = "map";

  // repeat until convergence
  for (int t = 0; t < max_iterations; ++t) {
    // transform source using estimated transformation
    pcl::transformPointCloud<PointT>(*source, *source_trans, final_transformation);

    // find correspondences in target
    std::vector<int> indices_src;
    std::vector<int> indices_tgt;
    for (int i = 0; i < cloud_size; ++i) {
      kdtree->nearestKSearch(source_trans->points[i], 1, indices, sq_dist);
      if (sq_dist[0] > max_distance * max_distance)  // skip if too far
        continue;
      indices_src.push_back(i);
      indices_tgt.push_back(indices[0]);
    }

    // copy selected correspondences to new point clouds
    pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_tgt(new pcl::PointCloud<PointT>);
    CopyPointCloud(source_trans, indices_src, cloud_src);
    CopyPointCloud(target, indices_tgt, cloud_tgt);

    // solve transformation using Gauss-Newton optimization
    int indices_size = static_cast<int>(indices_src.size());
    Eigen::VectorXd Residual(indices_size);
    Eigen::Matrix4d transformation = GaussNewton(cloud_src, cloud_tgt, Residual);
    final_transformation = transformation * final_transformation;
    std::cout << "iter = " << t << "; cloud size = " << cloud_size << "; indices_size = " << indices_size << std::endl;

    // prepare visualization data
    for (int i = 0; i < cloud_size; ++i) {
      source_trans->points[i].intensity = 0;
    }
    for (int i = 0; i < indices_size; ++i) {
      source_trans->points[indices_src[i]].intensity = std::abs(Residual.cast<float>()[i]);
    }
    correspondence_msg.index_source = indices_src;
    correspondence_msg.index_target = indices_tgt;
    pcl::toROSMsg(*source_trans, cloud_source);
    cloud_source.header.frame_id = "map";
    correspondence_msg.cloud_source = cloud_source;
    pub_correspondence_.publish(correspondence_msg);
    pub_source_cloud_.publish(cloud_source);
    pub_target_cloud_.publish(cloud_target);

    // check convergence
    double cos_angle = 0.5 * (transformation.coeff(0, 0) + transformation.coeff(1, 1) + transformation.coeff(2, 2) - 1);
    double translation_sqr = transformation.coeff(0, 3) * transformation.coeff(0, 3) +
                             transformation.coeff(1, 3) * transformation.coeff(1, 3) +
                             transformation.coeff(2, 3) * transformation.coeff(2, 3);
    if (cos_angle >= rotation_epsilon && translation_sqr <= translation_epsilon) {
      std::cout << "converged!" << std::endl;
      break;
    }
    if (!ros::ok()) break;
    rate.sleep();
  }

  return final_transformation;
}

Eigen::Matrix4d ICPVisualization::GaussNewton(const PointCloudPtr& source, const PointCloudPtr& target,
                                              Eigen::VectorXd& Residual) {
  // math formulation according to the paper: Colored Point Cloud Registration Revisited, ICCV 2017
  int size = static_cast<int>(source->size());
  Eigen::Matrix6Xd Jacobian(6, size);  // alpha, beta, gamma, a, b, c as in the linearized transformation matrix
  // Eigen::VectorXd Residual(size);      // see Eq. 20 in the paper

  for (int i = 0; i < size; ++i) {
    const Eigen::Vector3d q = source->points[i].getVector3fMap().cast<double>();
    const Eigen::Vector3d p = target->points[i].getVector3fMap().cast<double>();
    const Eigen::Vector3d np = target->points[i].getNormalVector3fMap().cast<double>();
    Residual(i) = (q - p).dot(np);             // Eq. 19 in the paper
    Jacobian.block<3, 1>(0, i) = q.cross(np);  // Eq. 30 in the paper
    Jacobian.block<3, 1>(3, i) = np;
  }

  Eigen::Matrix6d JTJ = Jacobian * Jacobian.transpose();  // Jacobian herein has already been transposed (row vector)
  Eigen::Vector6d JTr = Jacobian * Residual;
  Eigen::Vector6d X = JTJ.ldlt().solve(-JTr);  // solve a system of linear equations in the form: AX = b
  return TransformVector6dToMatrix4d(X);
}

Eigen::Matrix4d ICPVisualization::TransformVector6dToMatrix4d(const Eigen::Vector6d& input) {
  Eigen::Matrix4d output;
  output.setIdentity();
  // AngleAxis representation implicitly maps the linearized matrix to SO(3)
  output.block<3, 3>(0, 0) =
      (Eigen::AngleAxisd(input(2), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(input(1), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(input(0), Eigen::Vector3d::UnitX()))
          .matrix();
  output.block<3, 1>(0, 3) = input.block<3, 1>(3, 0);
  return output;
}

}  // namespace icp_vis
