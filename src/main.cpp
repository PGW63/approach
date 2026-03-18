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
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  roi_filter_ = std::make_shared<Filter>();
  plane_filter_ = std::make_shared<PlaneFilter>();
  pid_controller_ = std::make_shared<PIDController>();
  error_estimator_ = std::make_shared<ErrorEstimator>();
  edge_extractor_ = std::make_shared<EdgeExtractor>();

  this->declare_parameter<std::string>(
    "pointcloud_topic_name", "/camera/camera_head/depth/color/points");
  this->declare_parameter<std::string>(
    "detection_topic_name", "/approach/detector/detection_array");
  this->declare_parameter<std::string>(
    "info_topic_name", "/camera/camera_head/color/camera_info");
  this->declare_parameter<std::string>("target_frame", "base");
  this->declare_parameter<int>("sync_queue_size", 10);
  this->declare_parameter<float>("leaf_size", 0.01F);
  this->declare_parameter<int>("mean_k", 50);
  this->declare_parameter<float>("stddev_mul_thresh", 1.0F);
  this->declare_parameter<float>("ground_height", 0.05F);

  this->get_parameter("pointcloud_topic_name", pointcloud_topic_name_);
  this->get_parameter("detection_topic_name", detection_topic_name_);
  this->get_parameter("info_topic_name", info_topic_name_);
  this->get_parameter("target_frame", target_frame_);
  this->get_parameter("sync_queue_size", sync_queue_size_);
  this->get_parameter("leaf_size", leaf_size_);
  this->get_parameter("mean_k", mean_k_);
  this->get_parameter("stddev_mul_thresh", stddev_mul_thresh_);
  this->get_parameter("ground_height", ground_height_);

  camera_info_subscriber_ = this->create_subscription<CameraInfoMsg>(
    info_topic_name_,
    qos_best_effort_,
    std::bind(&ApproachNode::cameraInfoCallback, this, std::placeholders::_1));

  point_cloud_subscriber_.subscribe(
    this,
    pointcloud_topic_name_,
    rmw_qos_profile_sensor_data);
  detection_subscriber_.subscribe(
    this,
    detection_topic_name_,
    rmw_qos_profile_default);

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(sync_queue_size_),
    point_cloud_subscriber_,
    detection_subscriber_);
  sync_->registerCallback(
    std::bind(&ApproachNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    qos_reliable_);
  filtered_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/approach/filtered_pointcloud",
    qos_best_effort_);

  roi_filter_->setParameters(leaf_size_, mean_k_, stddev_mul_thresh_, ground_height_);
}

void ApproachNode::syncCallback(
  const PointCloudMsg::ConstSharedPtr & pointcloud_msg,
  const DetectionMsg::ConstSharedPtr & detection_msg)
{
  (void)pointcloud_msg;
  (void)detection_msg;

  if (!received_camera_info_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Waiting for camera info before processing synchronized messages.");
    return;
  }



  if (detection_msg->detections.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "No detections in synchronized message.");
    return;
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "Received synchronized pointcloud and detection array with %zu detections.",
    detection_msg->detections.size());
  
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  kdtree.reset(new pcl::search::KdTree<pcl::PointXYZ>);

  roi_filter_->roi_filter(pointcloud_msg, detection_msg, cloud);
  roi_filter_->voxel_downsampling(cloud);
  roi_filter_->remove_outliers(cloud);

  geometry_msgs::msg::TransformStamped tf;
  if (getTransform(target_frame_, pointcloud_msg->header.frame_id, tf, pointcloud_msg->header.stamp)) {
    RCLCPP_INFO(this->get_logger(), "TF received");
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to get TF. Skipping point cloud transformation and return");
    return;
  }
  roi_filter_->remove_ground(cloud, tf.transform);
  kdtree->setInputCloud(cloud);
  roi_filter_->cluster_points(cloud, kdtree);

  sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*cloud, filtered_cloud_msg);
  filtered_cloud_msg.header = pointcloud_msg->header;
  filtered_cloud_msg.header.frame_id = target_frame_; // Points are now in target_frame_
  filtered_pointcloud_publisher_->publish(filtered_cloud_msg);




}

void ApproachNode::cameraInfoCallback(const CameraInfoMsg::SharedPtr msg)
{
  if (received_camera_info_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received CameraInfo message");
  roi_filter_->setCameraInfo(
    msg->k[0],  // fx
    msg->k[4],  // fy
    msg->k[2],  // cx
    msg->k[5]   // cy
  );
  received_camera_info_ = true;
}

bool ApproachNode::getTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform,
  const rclcpp::Time & stamp)
{
  try {
    // stamp로 바꿀것!! 
    transform = tf_buffer_.lookupTransform(
      target_frame,
      source_frame,
      rclcpp::Time(0, 0, RCL_ROS_TIME),  // equivalent to latest available TF in ROS2
      rclcpp::Duration::from_seconds(0.1));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to get TF from %s to %s: %s, %f",
      source_frame.c_str(),
      target_frame.c_str(),
      ex.what(),
      stamp.seconds());
    return false;
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachNode>());
  rclcpp::shutdown();
  return 0;
}
