#pragma once

#include "approach/convexhull.hpp"
#include "approach/edge_extractor.hpp"
#include "approach/error_estimator.hpp"
#include "approach/pid_controller.hpp"
#include "approach/plane_filter.hpp"
#include "approach/roi_filter.hpp"

#include <atomic>
#include <memory>
#include <string>

#include "inha_interfaces/action/approach.hpp"
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
#include <rclcpp_action/rclcpp_action.hpp>
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
  using ApproachAction = inha_interfaces::action::Approach;
  using GoalHandleApproach = rclcpp_action::ServerGoalHandle<ApproachAction>;

  std::shared_ptr<Filter> roi_filter_;
  std::shared_ptr<ConvexHull> convex_hull_;
  std::shared_ptr<PlaneFilter> plane_filter_;
  std::shared_ptr<PIDController> pid_controller_;
  std::shared_ptr<ErrorEstimator> error_estimator_;
  std::shared_ptr<EdgeExtractor> edge_extractor_;

  rclcpp_action::Server<ApproachAction>::SharedPtr approach_action_server_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const ApproachAction::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleApproach>);
  void handle_accepted(const std::shared_ptr<GoalHandleApproach> goal_handle);
  void execute(const std::shared_ptr<GoalHandleApproach> goal_handle);

  void stopAlgorithm();
  void startAlgorithm();

  message_filters::Subscriber<PointCloudMsg> point_cloud_subscriber_;
  message_filters::Subscriber<DetectionMsg> detection_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Subscription<CameraInfoMsg>::SharedPtr camera_info_subscriber_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      filtered_pointcloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      debugging_pointcloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obb_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      target_edge_publisher_;

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
  float cluster_tolerance_ = 0.02F;
  int min_cluster_size_ = 100;
  int max_cluster_size_ = 10000;
  float fx_ = 0.0F;
  float fy_ = 0.0F;
  float cx_ = 0.0F;
  float cy_ = 0.0F;
  float kp_x_ = 0.25F;
  float kp_y_ = 2.0F;
  float kp_theta_ = 1.2F;
  float ki_x_ = 0.0F;
  float ki_y_ = 0.0F;
  float ki_theta_ = 0.0F;
  float kd_x_ = 0.0F;
  float kd_y_ = 0.0F;
  float kd_theta_ = 0.0F;
  float tol_x_ = 0.05F;
  float tol_y_ = 0.02F;
  float tol_theta_ = 0.08F;
  float base_to_rotationcore_ = 0.2F;

  std::string main_target;
  bool start_time_flag = false;
  rclcpp::Time start_time;

  std::atomic<bool> control_success = false;
  std::atomic<bool> control_failure = false;
  std::string failure_message;
  std::atomic<bool> algorithm_start_flag = false;

  OBB obb;
  SE2Error se2_error;
  SE2Error se2_error_prev;

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
  void publishTargetEdge(const TargetEdge &target_edge);
};
