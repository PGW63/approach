#include "approach/main.hpp"

#include <functional>
#include <memory>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>
#include <rmw/qos_profiles.h>

ApproachNode::ApproachNode()
    : Node("approach_node"),
      qos_best_effort_(rclcpp::QoS(rclcpp::KeepLast(10)).best_effort()),
      qos_reliable_(rclcpp::QoS(rclcpp::KeepLast(10)).reliable()),
      tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {

  /*
  객체 생성
  */
  roi_filter_ = std::make_shared<Filter>();
  convex_hull_ = std::make_shared<ConvexHull>();
  plane_filter_ = std::make_shared<PlaneFilter>();
  pid_controller_ = std::make_shared<PIDController>();
  edge_extractor_ = std::make_shared<EdgeExtractor>();
  error_estimator_ = std::make_shared<ErrorEstimator>();

  /*
  파라미터 선언
  */
  this->declare_parameter<std::string>(
      "pointcloud_topic_name", "/camera/camera_head/depth/color/points");
  this->declare_parameter<std::string>("detection_topic_name",
                                       "/approach/detector/detection_array");
  this->declare_parameter<std::string>("info_topic_name",
                                       "/camera/camera_head/color/camera_info");
  this->declare_parameter<std::string>("target_frame", "base");
  this->declare_parameter<int>("sync_queue_size", 10);
  this->declare_parameter<float>("leaf_size", 0.01F);
  this->declare_parameter<int>("mean_k", 50);
  this->declare_parameter<float>("stddev_mul_thresh", 0.5F);
  this->declare_parameter<float>("ground_height", 0.02F);
  this->declare_parameter<float>("cluster_tolerance", 0.02F);
  this->declare_parameter<int>("min_cluster_size", 100);
  this->declare_parameter<int>("max_cluster_size", 10000);
  this->declare_parameter<float>("fx", 605.7842407226562F);
  this->declare_parameter<float>("fy", 604.492919921875F);
  this->declare_parameter<float>("cx", 323.7266845703125F);
  this->declare_parameter<float>("cy", 250.73828125F);
  this->declare_parameter<float>("kp_x", 0.25F);
  this->declare_parameter<float>("kp_y", 2.0F);
  this->declare_parameter<float>("kp_theta", 1.2F);
  this->declare_parameter<float>("ki_x", 0.0F);
  this->declare_parameter<float>("ki_y", 0.0F);
  this->declare_parameter<float>("ki_theta", 0.0F);
  this->declare_parameter<float>("kd_x", 0.0F);
  this->declare_parameter<float>("kd_y", 0.0F);
  this->declare_parameter<float>("kd_theta", 0.0F);
  this->declare_parameter<float>("tol_x", 0.05F);     // 5cm
  this->declare_parameter<float>("tol_y", 0.02F);     // 2cm
  this->declare_parameter<float>("tol_theta", 0.08F); // 4~5도
  this->declare_parameter<float>("base_to_rotationcore", 0.2F);

  this->get_parameter("pointcloud_topic_name", pointcloud_topic_name_);
  this->get_parameter("detection_topic_name", detection_topic_name_);
  this->get_parameter("info_topic_name", info_topic_name_);
  this->get_parameter("target_frame", target_frame_);
  this->get_parameter("sync_queue_size", sync_queue_size_);
  this->get_parameter("leaf_size", leaf_size_);
  this->get_parameter("mean_k", mean_k_);
  this->get_parameter("stddev_mul_thresh", stddev_mul_thresh_);
  this->get_parameter("ground_height", ground_height_);
  this->get_parameter("cluster_tolerance", cluster_tolerance_);
  this->get_parameter("min_cluster_size", min_cluster_size_);
  this->get_parameter("max_cluster_size", max_cluster_size_);
  this->get_parameter("fx", fx_);
  this->get_parameter("fy", fy_);
  this->get_parameter("cx", cx_);
  this->get_parameter("cy", cy_);
  this->get_parameter("kp_x", kp_x_);
  this->get_parameter("kp_y", kp_y_);
  this->get_parameter("kp_theta", kp_theta_);
  this->get_parameter("ki_x", ki_x_);
  this->get_parameter("ki_y", ki_y_);
  this->get_parameter("ki_theta", ki_theta_);
  this->get_parameter("kd_x", kd_x_);
  this->get_parameter("kd_y", kd_y_);
  this->get_parameter("kd_theta", kd_theta_);
  this->get_parameter("tol_x", tol_x_);
  this->get_parameter("tol_y", tol_y_);
  this->get_parameter("tol_theta", tol_theta_);
  this->get_parameter("base_to_rotationcore", base_to_rotationcore_);

  /*
  ROS2 Publisher && Subscriber 설정
  */

  this->action_server_ = rclcpp_action::create_server<ApproachAction>(
      this, "approach",
      std::bind(&ApproachActionServer::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ApproachActionServer::handle_cancel, this,
                std::placeholders::_1),
      std::bind(&ApproachActionServer::handle_accepted, this,
                std::placeholders::_1, std::placeholders::_2));

  camera_info_subscriber_ = this->create_subscription<CameraInfoMsg>(
      info_topic_name_, qos_best_effort_,
      std::bind(&ApproachNode::cameraInfoCallback, this,
                std::placeholders::_1));

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size_), point_cloud_subscriber_,
      detection_subscriber_);
  sync_->registerCallback(std::bind(&ApproachNode::syncCallback, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos_reliable_);
  filtered_pointcloud_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/approach/filtered_pointcloud", qos_best_effort_);
  obb_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/approach/obb_marker", qos_reliable_);
  target_edge_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
          "/approach/target_edge_marker", qos_reliable_);
  debugging_pointcloud_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/approach/debugging_pointcloud", qos_best_effort_);

  /*
  파라미터설정
  */
  roi_filter_->setParameters(leaf_size_, mean_k_, stddev_mul_thresh_,
                             ground_height_, cluster_tolerance_,
                             min_cluster_size_, max_cluster_size_);
  pid_controller_->setParameters(kp_x_, kp_y_, kp_theta_, ki_x_, ki_y_,
                                 ki_theta_, kd_x_, kd_y_, kd_theta_, tol_x_,
                                 tol_y_, tol_theta_, base_to_rotationcore_);
}
/*
Ros2 Action 관련 함수들
*/
rclcpp_action::GoalResponse
ApproachNode::handle_goal(const rclcpp_action::GoalUUID &uuid,
                          std::shared_ptr<const ApproachAction::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received approach goal");
  main_target = goal->main_target;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ApproachNode::handle_cancel(
    const std::shared_ptr<GoalHandleApproach> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received approach cancel");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ApproachNode::handle_accepted(
    const std::shared_ptr<GoalHandleApproach> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Approach goal accepted");
  std::thread(std::bind(&ApproachNode::execute, this, goal_handle)).detach();
}

void ApproachNode::execute(
    const std::shared_ptr<GoalHandleApproach> goal_handle) {

  auto feedback = std::make_shared<ApproachAction::Feedback>();
  auto result = std::make_shared<ApproachAction::Result>();
  if (!algorithm_start_flag) {
    algorithm_start_flag = true;
    startAlgorithm();
  }

  rclcpp::Rate loop_rate(10); // 10Hz 제어 루프 속도 설정

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Approach goal canceled");
      goal_handle->abort(result);
      stopAlgorithm();
      return;
    }

    if (control_success) {
      RCLCPP_INFO(this->get_logger(), "Approach goal succeeded");
      result->success = true;
      result->success_message = "Approach goal succeeded";
      goal_handle->succeed(result);
      stopAlgorithm();
      return;
    }

    if (control_failure) {
      RCLCPP_INFO(this->get_logger(), "Approach goal failed");
      result->success = false;
      result->success_message = failure_message;
      goal_handle->abort(result);
      stopAlgorithm();
      return;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Executing approach");

    feedback->x_error = se2_error.x;
    feedback->y_error = se2_error.y;
    feedback->theta_error = se2_error.degree_theta;
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }
}
/*
콜백함수(메인 함수)
*/
void ApproachNode::syncCallback(
    const PointCloudMsg::ConstSharedPtr &pointcloud_msg,
    const DetectionMsg::ConstSharedPtr &detection_msg) {
  (void)pointcloud_msg;
  (void)detection_msg;

  if (!algorithm_start_flag) {
    return;
  }

  if (!received_camera_info_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for camera info before processing synchronized messages.");
    return;
  }

  if (detection_msg->detections.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "No detections in synchronized message.");
    control_failure = true;
    failure_message = "No detections in synchronized message.";
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
               "Received synchronized pointcloud and detection array with %zu "
               "detections.",
               detection_msg->detections.size());

  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  kdtree.reset(new pcl::search::KdTree<pcl::PointXYZ>);

  /*
  포인트 클라우드 전처리
  (디텍션을 통해 관심영역 설정 + 다운샘플링 + 이상치제거)
  */
  roi_filter_->roi_filter(pointcloud_msg, detection_msg, cloud);
  roi_filter_->voxel_downsampling(cloud);
  roi_filter_->remove_outliers(cloud);

  geometry_msgs::msg::TransformStamped tf;
  if (getTransform(target_frame_, pointcloud_msg->header.frame_id, tf,
                   pointcloud_msg->header.stamp)) {
    RCLCPP_INFO(this->get_logger(), "TF received");
  } else {
    RCLCPP_WARN(
        this->get_logger(),
        "Failed to get TF. Skipping point cloud transformation and return");
    return;
  }
  roi_filter_->remove_ground(cloud, tf.transform);

  // 디버깅용 포인트클라우드
  sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*cloud, filtered_cloud_msg);
  filtered_cloud_msg.header.stamp = this->now();
  filtered_cloud_msg.header.frame_id = target_frame_;
  debugging_pointcloud_publisher_->publish(filtered_cloud_msg);

  /*
  BBOX를 통해 관심영역을 설정하여서 뒤에 다른 물체들 혹은 여러 물체들이 잡혔을
  수도 있기에 클러스터링을 통해 가장 큰 클러스터만 남긴다.
  */
  kdtree->setInputCloud(cloud);
  roi_filter_->cluster_points(cloud, kdtree);

  publish3Dpointcloud(cloud);

  roi_filter_->projection_filter(cloud);
  roi_filter_->front_slicing(cloud);

  obb = plane_filter_->compute_OBB(cloud);
  publish2DOBB(obb.center, obb.axis1, obb.axis2, obb.length1, obb.length2);

  /*
  OBB에서 구한 사각형 중 로봇과 가장 가까우면서도 수평방향을 띄고 있는 축을
  내적을 통해 구한다.
  */
  TargetEdge target_edge = edge_extractor_->extract_edges(
      obb.center, obb.axis1, obb.axis2, obb.length1, obb.length2);
  publishTargetEdge(target_edge);

  // error estimator => SE(2) error 측정
  se2_error = error_estimator_->estimate_error(target_edge);

  /*
  pid controller => SE(2) -> 로봇의 모델 방정식 -> v_x, w_z 측정
  */
  rclcpp::Time current_time = this->now();
  if (previous_time_.nanoseconds() == 0) {
    previous_time_ = current_time;
  }
  float dt = (current_time - previous_time_).seconds();
  previous_time_ = current_time;

  // 허용 오차
  float tol_x = tol_x_;
  float tol_y = tol_y_;
  float tol_theta = tol_theta_;
  // 세 가지 오차가 모두 허용 범위 안에 들어왔다면 "도착"으로 판정!
  if (std::abs(se2_error.x) < tol_x && std::abs(se2_error.y) < tol_y &&
      std::abs(se2_error.degree_theta) < tol_theta) {

    RCLCPP_INFO(this->get_logger(), "목표 지점에 도착 완료!");
    if (start_time_flag == false) {
      start_time = current_time;
      start_time_flag = true;
    }
    if ((current_time - start_time).seconds() > 2.0) {
      control_success = true;
    }

    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_publisher_->publish(stop_msg);

    return;
  } else if (std::abs(se2_error.x) < tol_x && std::abs(se2_error.y) > tol_y) {
    RCLCPP_INFO(this->get_logger(), "종방향 수정 종료. 판단 후 Failure 발행");
    control_failure = true;
    failure_message = "종방향 수정 종료";
    return;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel = pid_controller_->compute_control(se2_error, dt);

  cmd_vel_publisher_->publish(cmd_vel);
}

void ApproachNode::cameraInfoCallback(const CameraInfoMsg::SharedPtr msg) {
  if (!algorithm_start_flag) {
    return;
  }

  if (received_camera_info_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received CameraInfo message");
  roi_filter_->setCameraInfo(msg->k[0], // fx
                             msg->k[4], // fy
                             msg->k[2], // cx
                             msg->k[5]  // cy
  );
  received_camera_info_ = true;
}

bool ApproachNode::getTransform(const std::string &target_frame,
                                const std::string &source_frame,
                                geometry_msgs::msg::TransformStamped &transform,
                                const rclcpp::Time &stamp) {
  try {
    /*
    Real World -> stamp
    Bag files or Simulation -> rclcpp::Time(0, 0, RCL_ROS_TIME)
    */
    transform = tf_buffer_.lookupTransform(target_frame, source_frame,
                                           stamp, // Here
                                           rclcpp::Duration::from_seconds(0.1));
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to get TF from %s to %s: %s, %f",
                source_frame.c_str(), target_frame.c_str(), ex.what(),
                stamp.seconds());
    return false;
  }
}

void ApproachNode::publish3Dpointcloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*cloud, filtered_cloud_msg);
  filtered_cloud_msg.header.stamp = this->now();
  filtered_cloud_msg.header.frame_id =
      target_frame_; // Points are now in target_frame_
  filtered_pointcloud_publisher_->publish(filtered_cloud_msg);
}

void ApproachNode::publish2DOBB(const Eigen::Vector2f &center,
                                const Eigen::Vector2f &axis1,
                                const Eigen::Vector2f &axis2,
                                const float length1, const float length2) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = target_frame_;
  marker.header.stamp = this->now();
  marker.ns = "obb";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = 0.05; // Line width

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  Eigen::Vector2f p1 =
      center + (length1 / 2.0f) * axis1 + (length2 / 2.0f) * axis2;
  Eigen::Vector2f p2 =
      center - (length1 / 2.0f) * axis1 + (length2 / 2.0f) * axis2;
  Eigen::Vector2f p3 =
      center - (length1 / 2.0f) * axis1 - (length2 / 2.0f) * axis2;
  Eigen::Vector2f p4 =
      center + (length1 / 2.0f) * axis1 - (length2 / 2.0f) * axis2;

  geometry_msgs::msg::Point pt;
  pt.z = ground_height_; // Place it on the ground height

  pt.x = p1.x();
  pt.y = p1.y();
  marker.points.push_back(pt);
  pt.x = p2.x();
  pt.y = p2.y();
  marker.points.push_back(pt);
  pt.x = p3.x();
  pt.y = p3.y();
  marker.points.push_back(pt);
  pt.x = p4.x();
  pt.y = p4.y();
  marker.points.push_back(pt);
  pt.x = p1.x();
  pt.y = p1.y();
  marker.points.push_back(pt); // Close the loop

  obb_publisher_->publish(marker);
}

void ApproachNode::publishTargetEdge(const TargetEdge &target_edge) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = target_frame_;
  marker.header.stamp = this->now();
  marker.ns = "target_edge";
  marker.id = 0;

  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.05;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  geometry_msgs::msg::Point pt;
  pt.z = ground_height_;
  Eigen::Vector2f p1 =
      target_edge.target_center +
      (target_edge.target_length / 2.0f) * target_edge.target_axis;
  Eigen::Vector2f p2 =
      target_edge.target_center -
      (target_edge.target_length / 2.0f) * target_edge.target_axis;
  pt.x = p1.x();
  pt.y = p1.y();
  marker.points.push_back(pt);
  pt.x = p2.x();
  pt.y = p2.y();
  marker.points.push_back(pt);
  Eigen::Vector2f p3_normal_end =
      target_edge.target_center - target_edge.normal_axis * 0.4f;
  pt.x = target_edge.target_center.x();
  pt.y = target_edge.target_center.y();
  marker.points.push_back(pt);

  pt.x = p3_normal_end.x();
  pt.y = p3_normal_end.y();
  marker.points.push_back(pt);
  target_edge_publisher_->publish(marker);
}

void ApproachNode::stopAlgorithm() {
  point_cloud_subscriber_.unsubscribe();
  detection_subscriber_.unsubscribe();

  start_time_flag = false;
  control_success = false;
  control_failure = false;
  algorithm_start_flag = false;
}

void ApproachNode::startAlgorithm() {
  point_cloud_subscriber_.subscribe(this, pointcloud_topic_name_,
                                    rmw_qos_profile_sensor_data);
  detection_subscriber_.subscribe(this, detection_topic_name_,
                                  rmw_qos_profile_default);

  start_time_flag = false;
  control_success = false;
  control_failure = false;

  algorithm_start_flag = true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachNode>());
  rclcpp::shutdown();
  return 0;
}
