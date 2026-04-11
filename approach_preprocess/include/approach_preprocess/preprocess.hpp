#pragma once

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace approach_preprocess
{

struct PreprocessConfig
{
  bool remove_nan_enable{true};
  bool downsample_enable{true};
  bool outlier_removal_enable{true};
  bool robot_filter_enable{true};
  bool ground_removal_enable{true};
  bool keep_ground_for_mapping{true};
  double voxel_leaf_size{0.07};
  double passthrough_robot_x_min{-0.3};
  double passthrough_robot_x_max{0.2};
  double passthrough_robot_y_min{-0.3};
  double passthrough_robot_y_max{0.3};
  double passthrough_ground_z_{0.1};
  int mean_k{20};
  double stddev_mul_thresh{1.0};
};

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const PreprocessConfig & config);

pcl::PointCloud<pcl::PointXYZ>::Ptr removeNaN(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const PreprocessConfig & config);

pcl::PointCloud<pcl::PointXYZ>::Ptr removeRobotPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const PreprocessConfig & config);

pcl::PointCloud<pcl::PointXYZ>::Ptr removeGroundPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const PreprocessConfig & config);

pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const PreprocessConfig & config);

}  // namespace approach_preprocess
