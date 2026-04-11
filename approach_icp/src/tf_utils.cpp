# include "approach_icp/tf_utils.hpp"

#include <pcl/common/transforms.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <stdexcept>

namespace approach_icp
{
namespace tf_utils
{

geometry_msgs::msg::TransformStamped lookupTransform(
  tf2_ros::Buffer & buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  const rclcpp::Duration & timeout,
  bool fallback_to_latest_on_extrapolation,
  double fallback_max_gap_sec)
{
  try {
    return buffer.lookupTransform(target_frame, source_frame, stamp, timeout);
  } catch (const tf2::ExtrapolationException &) {
    if (!fallback_to_latest_on_extrapolation) {
      throw;
    }

    const auto latest_transform = buffer.lookupTransform(
      target_frame,
      source_frame,
      rclcpp::Time(0, 0, stamp.get_clock_type()),
      timeout);

    if (
      latest_transform.header.stamp.sec == 0 &&
      latest_transform.header.stamp.nanosec == 0)
    {
      return latest_transform;
    }

    const rclcpp::Time latest_time(latest_transform.header.stamp, stamp.get_clock_type());
    const double gap_sec = std::abs((latest_time - stamp).seconds());
    if (gap_sec <= fallback_max_gap_sec) {
      return latest_transform;
    }
    throw;
  }
}

Eigen::Matrix4f transformToMatrix(
  const geometry_msgs::msg::TransformStamped & transform)
{
  return tf2::transformToEigen(transform.transform).matrix().cast<float>();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
  const Eigen::Matrix4f & transform_matrix)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*input_cloud, *output, transform_matrix);
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1U;
  output->is_dense = false;
  return output;
}

}
}
