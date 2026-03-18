#pragma once

#include <Eigen/Dense>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct OBB {
  Eigen::Vector2f center;
  Eigen::Vector2f axis1;
  Eigen::Vector2f axis2;
  float length1;
  float length2;
};

class PlaneFilter {
public:
  PlaneFilter() = default;

  OBB compute_OBB(pcl::PointCloud<pcl::PointXY>::Ptr &cloud);

private:
  pcl::PointXY min_point_OBB;
  pcl::PointXY max_point_OBB;
  pcl::PointXY position_OBB;
  Eigen::Matrix2f rotation_matrix_OBB;
};
