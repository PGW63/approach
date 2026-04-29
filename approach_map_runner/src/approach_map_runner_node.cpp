#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <yaml-cpp/yaml.h>

#include "approach_map/map.hpp"
#include "inha_interfaces/srv/mapping_control.hpp"
#include "inha_interfaces/srv/set_enable.hpp"
#include "approach_preprocess/preprocess.hpp"

namespace
{

struct RunnerConfig
{
  std::string input_cloud_topic{"/approach/accumulated_cloud"};
  std::string target_point_topic{"/approach/target_point"};
  std::string mapping_service_name{"approach_mapping"};
  std::string waiting_enable_service_name{"/approach/waiting/set_enable"};
  std::string map_frame_id{"map"};
  std::string obstacle_map_topic{"/approach/obstacle_map"};
  std::string nav2_obstacle_map_topic{"/approach/nav2_obstacle_map"};
  std::string feasible_map_topic{"/approach/feasible_map"};
  std::string nav2_feasible_map_topic{"/approach/nav2_feasible_map"};
  std::string clearance_map_topic{"/approach/clearance_map"};
  std::string transition_map_topic{"/approach/transition_count_map"};
  std::string robot_filter_frame_id{"base"};
  bool use_initial_origin{false};
  bool allow_origin_updates_after_first_click{false};
  double initial_origin_x_m{0.0};
  double initial_origin_y_m{0.0};
  double ground_z_min_m{-0.20};
  double ground_z_max_m{0.20};
  double obstacle_z_min_m{0.05};
  double obstacle_z_max_m{1.50};
  int publish_heading_bin{0};
  double clearance_display_cap_m{1.5};
  double transform_timeout_sec{0.1};
  bool preprocess_remove_nan_enable{true};
  bool preprocess_downsample_enable{true};
  bool preprocess_outlier_removal_enable{true};
  bool robot_filter_enable{true};
  double preprocess_voxel_leaf_size{0.07};
  int outlier_mean_k{20};
  double outlier_stddev_mul_thresh{1.0};
  double robot_filter_x_min{-0.3};
  double robot_filter_x_max{0.2};
  double robot_filter_y_min{-0.3};
  double robot_filter_y_max{0.3};
};

template<typename T>
T readRequired(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required runner config key: ") + key);
  }
  return node[key].as<T>();
}

template<typename T>
T readOptional(const YAML::Node & node, const char * key, const T & default_value)
{
  return node[key] ? node[key].as<T>() : default_value;
}

RunnerConfig loadRunnerConfig(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const YAML::Node runner = root["runner"] ? root["runner"] : root;

  RunnerConfig config;
  config.input_cloud_topic = readRequired<std::string>(runner, "input_cloud_topic");
  config.target_point_topic = runner["target_point_topic"] ?
    runner["target_point_topic"].as<std::string>() :
    readOptional<std::string>(runner, "clicked_point_topic", config.target_point_topic);
  config.mapping_service_name = readOptional<std::string>(
    runner, "mapping_service_name", config.mapping_service_name);
  config.waiting_enable_service_name = readOptional<std::string>(
    runner, "waiting_enable_service_name", config.waiting_enable_service_name);
  config.map_frame_id = readRequired<std::string>(runner, "map_frame_id");
  config.obstacle_map_topic = readOptional<std::string>(
    runner, "obstacle_map_topic", config.obstacle_map_topic);
  config.nav2_obstacle_map_topic = readOptional<std::string>(
    runner, "nav2_obstacle_map_topic", config.nav2_obstacle_map_topic);
  config.feasible_map_topic = readOptional<std::string>(
    runner, "feasible_map_topic", config.feasible_map_topic);
  config.nav2_feasible_map_topic = readOptional<std::string>(
    runner, "nav2_feasible_map_topic", config.nav2_feasible_map_topic);
  config.clearance_map_topic = readOptional<std::string>(
    runner, "clearance_map_topic", config.clearance_map_topic);
  config.transition_map_topic = readOptional<std::string>(
    runner, "transition_map_topic", config.transition_map_topic);
  config.robot_filter_frame_id =
    readOptional<std::string>(runner, "robot_filter_frame_id", config.robot_filter_frame_id);
  config.use_initial_origin = readRequired<bool>(runner, "use_initial_origin");
  config.allow_origin_updates_after_first_click =
    readOptional<bool>(runner, "allow_origin_updates_after_first_click", false);
  config.initial_origin_x_m = readRequired<double>(runner, "initial_origin_x_m");
  config.initial_origin_y_m = readRequired<double>(runner, "initial_origin_y_m");
  config.ground_z_min_m = readRequired<double>(runner, "ground_z_min_m");
  config.ground_z_max_m = readRequired<double>(runner, "ground_z_max_m");
  config.obstacle_z_min_m = readRequired<double>(runner, "obstacle_z_min_m");
  config.obstacle_z_max_m = readRequired<double>(runner, "obstacle_z_max_m");
  config.publish_heading_bin = readRequired<int>(runner, "publish_heading_bin");
  config.clearance_display_cap_m = readRequired<double>(runner, "clearance_display_cap_m");
  config.transform_timeout_sec = readRequired<double>(runner, "transform_timeout_sec");
  config.preprocess_remove_nan_enable =
    readOptional<bool>(runner, "preprocess_remove_nan_enable", config.preprocess_remove_nan_enable);
  config.preprocess_downsample_enable =
    readOptional<bool>(runner, "preprocess_downsample_enable", config.preprocess_downsample_enable);
  config.preprocess_outlier_removal_enable =
    readOptional<bool>(
    runner, "preprocess_outlier_removal_enable", config.preprocess_outlier_removal_enable);
  config.robot_filter_enable =
    readOptional<bool>(runner, "robot_filter_enable", config.robot_filter_enable);
  config.preprocess_voxel_leaf_size =
    readOptional<double>(runner, "preprocess_voxel_leaf_size", config.preprocess_voxel_leaf_size);
  config.outlier_mean_k = readOptional<int>(runner, "outlier_mean_k", config.outlier_mean_k);
  config.outlier_stddev_mul_thresh =
    readOptional<double>(runner, "outlier_stddev_mul_thresh", config.outlier_stddev_mul_thresh);
  config.robot_filter_x_min =
    readOptional<double>(runner, "robot_filter_x_min", config.robot_filter_x_min);
  config.robot_filter_x_max =
    readOptional<double>(runner, "robot_filter_x_max", config.robot_filter_x_max);
  config.robot_filter_y_min =
    readOptional<double>(runner, "robot_filter_y_min", config.robot_filter_y_min);
  config.robot_filter_y_max =
    readOptional<double>(runner, "robot_filter_y_max", config.robot_filter_y_max);
  return config;
}

approach_preprocess::PreprocessConfig makePreprocessConfig(const RunnerConfig & runner_config)
{
  approach_preprocess::PreprocessConfig config;
  config.remove_nan_enable = runner_config.preprocess_remove_nan_enable;
  config.downsample_enable = runner_config.preprocess_downsample_enable;
  config.outlier_removal_enable = runner_config.preprocess_outlier_removal_enable;
  config.robot_filter_enable = runner_config.robot_filter_enable;
  config.voxel_leaf_size = runner_config.preprocess_voxel_leaf_size;
  config.mean_k = runner_config.outlier_mean_k;
  config.stddev_mul_thresh = runner_config.outlier_stddev_mul_thresh;
  config.passthrough_robot_x_min = runner_config.robot_filter_x_min;
  config.passthrough_robot_x_max = runner_config.robot_filter_x_max;
  config.passthrough_robot_y_min = runner_config.robot_filter_y_min;
  config.passthrough_robot_y_max = runner_config.robot_filter_y_max;
  return config;
}

nav_msgs::msg::OccupancyGrid makeBaseGrid(
  const approach_map::GridMeta & meta,
  const std_msgs::msg::Header & header)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header = header;
  grid.info.resolution = static_cast<float>(meta.resolution_m);
  grid.info.width = static_cast<uint32_t>(meta.width_cells);
  grid.info.height = static_cast<uint32_t>(meta.height_cells);
  grid.info.origin.position.x = meta.origin_x_m;
  grid.info.origin.position.y = meta.origin_y_m;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(meta.width_cells * meta.height_cells, -1);
  return grid;
}

nav_msgs::msg::OccupancyGrid toOccupancyGrid(
  const approach_map::GridDataI8 & layer,
  const std_msgs::msg::Header & header)
{
  auto grid = makeBaseGrid(layer.meta, header);
  grid.data = layer.values;
  return grid;
}

nav_msgs::msg::OccupancyGrid toNav2OccupancyGrid(
  const approach_map::GridDataI8 & layer,
  const std_msgs::msg::Header & header)
{
  auto grid = makeBaseGrid(layer.meta, header);

  for (std::size_t i = 0; i < layer.values.size(); ++i) {
    const int8_t value = layer.values[i];
    if (value < 0) {
      continue;
    }

    if (value >= 100) {
      grid.data[i] = 100;
    } else if (value <= 0) {
      grid.data[i] = 0;
    } else {
      grid.data[i] = -1;
    }
  }

  return grid;
}

nav_msgs::msg::OccupancyGrid toNav2FeasibleOccupancyGrid(
  const approach_map::GridDataI8 & layer,
  const std_msgs::msg::Header & header)
{
  auto grid = makeBaseGrid(layer.meta, header);

  for (std::size_t i = 0; i < layer.values.size(); ++i) {
    const int8_t value = layer.values[i];
    if (value < 0) {
      continue;
    }

    if (value >= 100) {
      grid.data[i] = 0;
    } else if (value <= 0) {
      grid.data[i] = 100;
    } else {
      grid.data[i] = -1;
    }
  }

  return grid;
}

nav_msgs::msg::OccupancyGrid toClearanceGrid(
  const approach_map::GridDataF32 & layer,
  const std_msgs::msg::Header & header,
  double display_cap_m)
{
  auto grid = makeBaseGrid(layer.meta, header);
  const double safe_cap = std::max(display_cap_m, layer.meta.resolution_m);

  for (std::size_t i = 0; i < layer.values.size(); ++i) {
    if (layer.values[i] < 0.0F || !std::isfinite(layer.values[i])) {
      continue;
    }

    const double normalized =
      std::clamp(static_cast<double>(layer.values[i]) / safe_cap, 0.0, 1.0);
    grid.data[i] = static_cast<int8_t>(std::round(normalized * 100.0));
  }

  return grid;
}

nav_msgs::msg::OccupancyGrid toTransitionGrid(
  const approach_map::GridMeta & meta,
  const std::vector<uint32_t> & transition_counts,
  const std::vector<uint8_t> & observed_mask,
  const std_msgs::msg::Header & header)
{
  auto grid = makeBaseGrid(meta, header);
  if (transition_counts.size() != grid.data.size() || observed_mask.size() != grid.data.size()) {
    return grid;
  }

  uint32_t max_transition_count = 0U;
  for (std::size_t i = 0; i < transition_counts.size(); ++i) {
    if (observed_mask[i] == 0U) {
      continue;
    }
    max_transition_count = std::max(max_transition_count, transition_counts[i]);
  }

  for (std::size_t i = 0; i < transition_counts.size(); ++i) {
    if (observed_mask[i] == 0U) {
      continue;
    }

    if (max_transition_count == 0U) {
      grid.data[i] = 0;
      continue;
    }

    const double normalized = static_cast<double>(transition_counts[i]) /
      static_cast<double>(max_transition_count);
    grid.data[i] = static_cast<int8_t>(std::round(std::clamp(normalized, 0.0, 1.0) * 100.0));
  }

  return grid;
}

class ApproachMapRunnerNode : public rclcpp::Node
{
public:
  using MappingControl = inha_interfaces::srv::MappingControl;
  using SetEnable = inha_interfaces::srv::SetEnable;

  ApproachMapRunnerNode()
  : Node("approach_map_runner_node")
  {
    const auto approach_share = ament_index_cpp::get_package_share_directory("approach_mapping");
    const auto runner_share = ament_index_cpp::get_package_share_directory("approach_map_runner");

    const std::string map_config_path = approach_share + "/config/map_config.yaml";
    const std::string runner_config_path = runner_share + "/config/runner_config.yaml";

    map_config_ = approach_map::loadConfigFromYaml(map_config_path);
    runner_config_ = loadRunnerConfig(runner_config_path);
    preprocess_config_ = makePreprocessConfig(runner_config_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const approach_map::Origin initial_origin{
      runner_config_.initial_origin_x_m,
      runner_config_.initial_origin_y_m};

    builder_ = std::make_unique<approach_map::Builder>(map_config_, initial_origin);
    origin_ready_ = runner_config_.use_initial_origin;

    obstacle_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      runner_config_.obstacle_map_topic, 1);
    nav2_obstacle_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      runner_config_.nav2_obstacle_map_topic, 1);
    nav2_feasible_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      runner_config_.nav2_feasible_map_topic, 1);
    clearance_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      runner_config_.clearance_map_topic, 1);
    feasible_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      runner_config_.feasible_map_topic, 1);
    transition_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      runner_config_.transition_map_topic, 1);
    auto target_point_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    target_point_qos.reliable();
    target_point_qos.transient_local();
    target_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      runner_config_.target_point_topic, target_point_qos);
    mapping_service_ = this->create_service<MappingControl>(
      runner_config_.mapping_service_name,
      std::bind(
        &ApproachMapRunnerNode::mappingServiceCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    waiting_enable_client_ = this->create_client<SetEnable>(
      runner_config_.waiting_enable_service_name);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      runner_config_.input_cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&ApproachMapRunnerNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Loaded map config: %s", map_config_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Loaded runner config: %s", runner_config_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Mapping frame: %s", runner_config_.map_frame_id.c_str());
    RCLCPP_INFO(
      this->get_logger(), "Mapping service: %s", runner_config_.mapping_service_name.c_str());
    RCLCPP_INFO(
      this->get_logger(), "Target point topic: %s", runner_config_.target_point_topic.c_str());
  }

private:
  tf2::Duration transformTimeout() const
  {
    return tf2::durationFromSec(std::max(0.0, runner_config_.transform_timeout_sec));
  }

  void publishTargetPoint(double x_m, double y_m)
  {
    geometry_msgs::msg::PointStamped target_msg;
    target_msg.header.stamp = this->now();
    target_msg.header.frame_id = runner_config_.map_frame_id;
    target_msg.point.x = x_m;
    target_msg.point.y = y_m;
    target_msg.point.z = 0.0;
    target_point_pub_->publish(target_msg);
  }

  void resetOriginFromTarget(double x_m, double y_m)
  {
    const approach_map::Origin origin{
      x_m - 0.5 * map_config_.width_m,
      y_m - 0.5 * map_config_.height_m};

    builder_->setOrigin(origin);
    origin_ready_ = true;

    RCLCPP_INFO(
      this->get_logger(), "Map origin reset from service target in %s: origin=(%.3f, %.3f)",
      runner_config_.map_frame_id.c_str(), origin.x_m, origin.y_m);
  }

  void mappingServiceCallback(
    const std::shared_ptr<MappingControl::Request> request,
    std::shared_ptr<MappingControl::Response> response)
  {
    if (!request->start) {
      mapping_enabled_ = false;
      origin_ready_ = false;
      RCLCPP_INFO(this->get_logger(), "Mapping stopped by service request.");
      callWaitingEnable(false);
      response->success = true;
      return;
    }

    if (request->mode != 0) {
      RCLCPP_WARN(
        this->get_logger(), "Unsupported mapping mode: %d. Only mode 0 is implemented.",
        request->mode);
      response->success = false;
      return;
    }

    if (request->target.size() < 2U) {
      RCLCPP_WARN(
        this->get_logger(),
        "Mapping start request requires target[0] and target[1], but received %zu values.",
        request->target.size());
      response->success = false;
      return;
    }

    const double target_x_m = static_cast<double>(request->target[0]);
    const double target_y_m = static_cast<double>(request->target[1]);
    if (!std::isfinite(target_x_m) || !std::isfinite(target_y_m)) {
      RCLCPP_WARN(
        this->get_logger(), "Mapping start request contains non-finite target coordinates.");
      response->success = false;
      return;
    }

    resetOriginFromTarget(target_x_m, target_y_m);
    publishTargetPoint(target_x_m, target_y_m);
    mapping_enabled_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Mapping started: mode=%d target=(%.3f, %.3f)%s",
      request->mode, target_x_m, target_y_m,
      request->target.size() > 2U ? " (additional target values ignored)" : "");
    callWaitingEnable(true);
    response->success = true;
  }

  void callWaitingEnable(bool enable)
  {
    if (!waiting_enable_client_->service_is_ready()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Waiting enable service %s not ready; skipping call.",
        runner_config_.waiting_enable_service_name.c_str());
      return;
    }

    auto request = std::make_shared<SetEnable::Request>();
    request->enable = enable;

    waiting_enable_client_->async_send_request(
      request,
      [this, enable](rclcpp::Client<SetEnable>::SharedFuture future) {
        const auto resp = future.get();
        RCLCPP_INFO(
          this->get_logger(),
          "Waiting enable(%d) response: success=%d message=%s",
          static_cast<int>(enable), static_cast<int>(resp->success),
          resp->message.c_str());
      });
  }

  bool transformCloud(
    const sensor_msgs::msg::PointCloud2 & input,
    const std::string & target_frame,
    sensor_msgs::msg::PointCloud2 & output)
  {
    if (input.header.frame_id == target_frame) {
      output = input;
      return true;
    }

    try {
      output = tf_buffer_->transform(input, target_frame, transformTimeout());
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Failed to transform cloud from %s to %s: %s",
        input.header.frame_id.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!mapping_enabled_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for mapping start service request on %s.",
        runner_config_.mapping_service_name.c_str());
      return;
    }

    if (!origin_ready_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for a valid mapping target before mapping.");
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_in_robot_filter_frame;
    if (!transformCloud(*msg, runner_config_.robot_filter_frame_id, cloud_in_robot_filter_frame)) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_in_robot_filter_frame, *processed_cloud);

    // if (preprocess_config_.remove_nan_enable && !processed_cloud->empty()) {
    //   processed_cloud = approach_preprocess::removeNaN(processed_cloud, preprocess_config_);
    // }
    // if (preprocess_config_.robot_filter_enable && !processed_cloud->empty()) {
    //   processed_cloud = approach_preprocess::removeRobotPoints(processed_cloud, preprocess_config_);
    // }
    // if (preprocess_config_.outlier_removal_enable && !processed_cloud->empty()) {
    //   processed_cloud = approach_preprocess::removeOutliers(processed_cloud, preprocess_config_);
    // }
    // if (preprocess_config_.downsample_enable && !processed_cloud->empty()) {
    //   processed_cloud = approach_preprocess::downsample(processed_cloud, preprocess_config_);
    // }

    sensor_msgs::msg::PointCloud2 filtered_cloud;
    pcl::toROSMsg(*processed_cloud, filtered_cloud);
    filtered_cloud.header = cloud_in_robot_filter_frame.header;

    sensor_msgs::msg::PointCloud2 transformed_cloud;
    if (!transformCloud(filtered_cloud, runner_config_.map_frame_id, transformed_cloud)) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(transformed_cloud, *map_cloud);

    builder_->beginUpdate();

    for (const auto & point : map_cloud->points) {
      const double x_m = point.x;
      const double y_m = point.y;
      const double z_m = point.z;

      if (!std::isfinite(x_m) || !std::isfinite(y_m) || !std::isfinite(z_m)) {
        continue;
      }

      if (z_m >= runner_config_.ground_z_min_m && z_m <= runner_config_.ground_z_max_m) {
        builder_->addGroundObservation(x_m, y_m);
      }

      if (z_m >= runner_config_.obstacle_z_min_m && z_m <= runner_config_.obstacle_z_max_m) {
        builder_->addObstacleObservation(x_m, y_m);
      }
    }

    builder_->endUpdate();

    std_msgs::msg::Header header = transformed_cloud.header;
    header.frame_id = runner_config_.map_frame_id;

    const auto obstacle_layer = builder_->buildObstacleLayer();
    const auto feasible_layer = builder_->buildHeadingFeasibleLayer(
      static_cast<std::size_t>(std::max(0, runner_config_.publish_heading_bin)));
    obstacle_pub_->publish(toOccupancyGrid(obstacle_layer, header));
    nav2_obstacle_map_pub_->publish(toNav2OccupancyGrid(obstacle_layer, header));
    nav2_feasible_map_pub_->publish(toNav2FeasibleOccupancyGrid(feasible_layer, header));
    clearance_pub_->publish(toClearanceGrid(
      builder_->buildClearanceLayer(), header, runner_config_.clearance_display_cap_m));
    feasible_pub_->publish(toOccupancyGrid(feasible_layer, header));
    transition_pub_->publish(toTransitionGrid(
      builder_->gridMeta(), builder_->stateTransitionCounts(), builder_->observedMask(), header));
  }

  approach_map::Config map_config_{};
  approach_preprocess::PreprocessConfig preprocess_config_{};
  RunnerConfig runner_config_{};
  std::unique_ptr<approach_map::Builder> builder_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool origin_ready_{false};
  bool mapping_enabled_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr nav2_obstacle_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr nav2_feasible_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr clearance_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr feasible_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr transition_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_point_pub_;
  rclcpp::Service<MappingControl>::SharedPtr mapping_service_;
  rclcpp::Client<SetEnable>::SharedPtr waiting_enable_client_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachMapRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
