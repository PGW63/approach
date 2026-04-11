#pragma once

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "approach_icp/types.hpp"

namespace approach_icp{

namespace tf_utils{

geometry_msgs::msg::TransformStamped lookupTransform(
    tf2_ros::Buffer& buffer,
    const std::string& target_frame,
    const std::string& source_frame,
    const rclcpp::Time& time,
    const rclcpp::Duration& timeout,
    bool fallback_to_latest = false,
    double fallback_max_gap_sec = 0.0
);

Eigen::Matrix4f transformToMatrix(
    const geometry_msgs::msg::TransformStamped& transform
);


pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    const Eigen::Matrix4f& transform
);
}
}
