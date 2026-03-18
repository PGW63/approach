#include "approach/plane_filter.hpp"

PlaneFilter::PlaneFilter() = default;

OBB PlaneFilter::compute_OBB(pcl::PointCloud<pcl::PointXY>::Ptr &cloud) {
  pcl::MomentOfInertiaEstimation<pcl::PointXY> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();

  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
                           rotation_matrix_OBB);

  OBB obb;
  obb.center[0] = position_OBB.x;
  obb.center[1] = position_OBB.y;
  obb.axis1[0] = rotation_matrix_OBB(0, 0);
  obb.axis1[1] = rotation_matrix_OBB(1, 0);
  obb.axis2[0] = rotation_matrix_OBB(0, 1);
  obb.axis2[1] = rotation_matrix_OBB(1, 1);
  obb.length1 = max_point_OBB.x - min_point_OBB.x;
  obb.length2 = max_point_OBB.y - min_point_OBB.y;

  return obb;
}