#pragma once

#include "approach/convexhull.hpp"
#include "approach/edge_extractor.hpp"
#include "approach/error_estimator.hpp"
#include "approach/pid_controller.hpp"
#include "approach/plane_filter.hpp"
#include "approach/roi_filter.hpp"

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

class ApproachNode : public rclcpp::Node {
public:
  ApproachNode();

private:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
  using DetectionMsg = vision_msgs::msg::Detection2DArray;
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<PointCloudMsg,
                                                      DetectionMsg>;

  std::shared_ptr<Filter> roi_filter_;
  std::shared_ptr<ConvexHull> convex_hull_;
  std::shared_ptr<PlaneFilter> plane_filter_;
  std::shared_ptr<PIDController> pid_controller_;
  std::shared_ptr<ErrorEstimator> error_estimator_;
  std::shared_ptr<EdgeExtractor> edge_extractor_;

  message_filters::Subscriber<PointCloudMsg> point_cloud_subscriber_;
  message_filters::Subscriber<DetectionMsg> detection_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Subscription<CameraInfoMsg>::SharedPtr camera_info_subscriber_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      filtered_pointcloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obb_publisher_;

  rclcpp::QoS qos_best_effort_;
  rclcpp::QoS qos_reliable_;

  std::string pointcloud_topic_name_;
  std::string detection_topic_name_;
  std::string info_topic_name_;
  std::string target_frame_;
  bool received_camera_info_ = false;
  int sync_queue_size_ = 10;
  float leaf_size_ = 0.02F;
  int mean_k_ = 50;
  float stddev_mul_thresh_ = 1.0F;
  float ground_height_ = 0.0F;

  OBB obb;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void syncCallback(const PointCloudMsg::ConstSharedPtr &pointcloud_msg,
                    const DetectionMsg::ConstSharedPtr &detection_msg);
  void cameraInfoCallback(const CameraInfoMsg::SharedPtr msg);
  bool getTransform(const std::string &target_frame,
                    const std::string &source_frame,
                    geometry_msgs::msg::TransformStamped &transform,
                    const rclcpp::Time &stamp = rclcpp::Time(0));

  void publish3Dpointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
  void publish2DOBB(const Eigen::Vector2f &center, const Eigen::Vector2f &axis1,
                    const Eigen::Vector2f &axis2, const float length1,
                    const float length2);
};
