#include "approach/plane_filter.hpp"

OBB PlaneFilter::compute_OBB(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  OBB obb;
  if (cloud->empty()) {
    obb.length1 = 0;
    obb.length2 = 0;
    return obb;
  }

  // 1. Convert PCL cloud to OpenCV 2D points (ignoring Z)
  std::vector<cv::Point2f> cv_points;
  cv_points.reserve(cloud->points.size());
  for (const auto &pt : cloud->points) {
    cv_points.emplace_back(pt.x, pt.y);
  }

  // 2. Compute the Minimum Area Bounding Rectangle
  cv::RotatedRect rect = cv::minAreaRect(cv_points);

  obb.center[0] = rect.center.x;
  obb.center[1] = rect.center.y;

  // 3. Extract 4 corners of the rotated rectangle
  cv::Point2f corners[4];
  rect.points(corners);

  // 4. Determine axes and lengths
  // corners are typically ordered clockwise or counter-clockwise
  cv::Point2f diff1 = corners[1] - corners[0];
  cv::Point2f diff2 = corners[2] - corners[1];

  float len1 = cv::norm(diff1);
  float len2 = cv::norm(diff2);

  obb.length1 = len1;
  obb.length2 = len2;

  if (len1 > 1e-5f) {
    obb.axis1[0] = diff1.x / len1;
    obb.axis1[1] = diff1.y / len1;
  } else {
    obb.axis1 = Eigen::Vector2f(1.0f, 0.0f);
  }

  if (len2 > 1e-5f) {
    obb.axis2[0] = diff2.x / len2;
    obb.axis2[1] = diff2.y / len2;
  } else {
    obb.axis2 = Eigen::Vector2f(0.0f, 1.0f);
  }

  return obb;
}