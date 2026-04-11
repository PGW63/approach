#pragma once

#include <Eigen/Core>
#include "approach_icp/types.hpp"
#include "approach_icp/tf_utils.hpp"
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>

namespace approach_icp
{
namespace registration
{

RegistrationResult registerPointClouds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess,
    const RegistrationConfig &config
);

}
}
