#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
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

  OBB compute_OBB(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

private:
};
