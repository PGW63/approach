#include "approach_preprocess/preprocess.hpp"

#include <limits>
#include <vector>

#include <pcl/filters/filter.h>

namespace approach_preprocess
{

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double voxel_leaf_size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(
    static_cast<float>(voxel_leaf_size),
    static_cast<float>(voxel_leaf_size),
    static_cast<float>(voxel_leaf_size));
  voxel_filter.filter(*downsampled);
  return downsampled;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const PreprocessConfig & config)
{
  return downsample(cloud, config.voxel_leaf_size);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeNaN(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const PreprocessConfig & config)
{
  (void)config;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *filtered, indices);
  return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeRobotPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const PreprocessConfig & config)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  filtered->reserve(cloud->size());

  for (const auto & point : cloud->points) {
    const bool inside_robot_x =
      point.x >= config.passthrough_robot_x_min &&
      point.x <= config.passthrough_robot_x_max;
    const bool inside_robot_y =
      point.y >= config.passthrough_robot_y_min &&
      point.y <= config.passthrough_robot_y_max;

    if (inside_robot_x && inside_robot_y) {
      continue;
    }

    filtered->push_back(point);
  }

  filtered->width = static_cast<std::uint32_t>(filtered->size());
  filtered->height = 1U;
  filtered->is_dense = false;

  return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeGroundPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const PreprocessConfig & config)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(config.passthrough_ground_z_, std::numeric_limits<float>::max());
  pass.filter(*filtered);
  return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const PreprocessConfig & config)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(config.mean_k);
  sor.setStddevMulThresh(config.stddev_mul_thresh);
  sor.filter(*filtered);
  return filtered;
}

}  // namespace approach_preprocess
