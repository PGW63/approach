#include "approach/convexhull.hpp"

ConvexHull::ConvexHull() = default;

void ConvexHull::compute(pcl::PointCloud<pcl::PointXY>::Ptr &cloud) {
  pcl::PointCloud<pcl::PointXY>::Ptr new_cloud(
      new pcl::PointCloud<pcl::PointXY>);
  pcl::ConvexHull<pcl::PointXY> chull;
  chull.setInputCloud(cloud);
  chull.reconstruct(*new_cloud);
  cloud = new_cloud;
}