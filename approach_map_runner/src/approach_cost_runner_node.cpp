#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "approach_cost/cost.hpp"
#include "inha_interfaces/msg/grasp_approach_status.hpp"

namespace
{

struct CostRunnerConfig
{
  std::string feasible_map_topic{"feasible_map"};
  std::string transition_map_topic{"transition_count_map"};
  std::string target_point_topic{"/approach/target_point"};
  std::string grasp_targets_topic{"/approach/grasp_targets"};
  std::string grasp_status_topic{"/approach/grasp_status"};
  std::string robot_frame_id{"base"};
  std::string output_topic{"final_cost_map"};
  std::string arrow_topic{"/approach/best_cost_arrow"};
  double transform_timeout_sec{0.1};
  double grasp_radius_m{0.9};
  bool debug_save_enable{false};
  std::string debug_output_dir{"/tmp/approach_cost_debug"};
  int debug_save_every_n_frames{1};
  int debug_max_frames_per_session{600};
};

CostRunnerConfig loadCostRunnerConfig(rclcpp::Node & node)
{
  CostRunnerConfig config;
  config.feasible_map_topic =
    node.declare_parameter("feasible_map_topic", config.feasible_map_topic);
  config.transition_map_topic =
    node.declare_parameter("transition_map_topic", config.transition_map_topic);
  config.target_point_topic =
    node.declare_parameter("target_point_topic", config.target_point_topic);
  config.grasp_targets_topic =
    node.declare_parameter("grasp_targets_topic", config.grasp_targets_topic);
  config.grasp_status_topic =
    node.declare_parameter("grasp_status_topic", config.grasp_status_topic);
  config.robot_frame_id =
    node.declare_parameter("robot_frame_id", config.robot_frame_id);
  config.output_topic = node.declare_parameter("output_topic", config.output_topic);
  config.arrow_topic = node.declare_parameter("arrow_topic", config.arrow_topic);
  config.transform_timeout_sec =
    node.declare_parameter("transform_timeout_sec", config.transform_timeout_sec);
  config.grasp_radius_m = node.declare_parameter("grasp_radius_m", config.grasp_radius_m);
  config.debug_save_enable =
    node.declare_parameter("debug_save_enable", config.debug_save_enable);
  config.debug_output_dir =
    node.declare_parameter("debug_output_dir", config.debug_output_dir);
  config.debug_save_every_n_frames = std::max(
    1, node.declare_parameter(
      "debug_save_every_n_frames", config.debug_save_every_n_frames));
  config.debug_max_frames_per_session = std::max(
    0, node.declare_parameter(
      "debug_max_frames_per_session", config.debug_max_frames_per_session));
  return config;
}

approach_map::GridMeta metaFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid & grid)
{
  approach_map::GridMeta meta;
  meta.width_cells = grid.info.width;
  meta.height_cells = grid.info.height;
  meta.resolution_m = grid.info.resolution;
  meta.origin_x_m = grid.info.origin.position.x;
  meta.origin_y_m = grid.info.origin.position.y;
  return meta;
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

nav_msgs::msg::OccupancyGrid toCostGrid(
  const approach_map::GridDataF32 & layer,
  const std_msgs::msg::Header & header)
{
  auto grid = makeBaseGrid(layer.meta, header);

  for (std::size_t i = 0; i < layer.values.size(); ++i) {
    const float value = layer.values[i];
    if (value < 0.0F || !std::isfinite(value)) {
      continue;
    }

    grid.data[i] = static_cast<int8_t>(
      std::round(std::clamp(static_cast<double>(value), 0.0, 1.0) * 100.0));
  }

  return grid;
}

std::vector<uint8_t> candidateMaskFromFeasibleMap(const nav_msgs::msg::OccupancyGrid & grid)
{
  std::vector<uint8_t> mask(grid.data.size(), 0U);
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    mask[i] = grid.data[i] > 0 ? 1U : 0U;
  }
  return mask;
}

std::vector<uint32_t> transitionCountsFromGrid(const nav_msgs::msg::OccupancyGrid & grid)
{
  std::vector<uint32_t> counts(grid.data.size(), 0U);
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    counts[i] = grid.data[i] > 0 ? static_cast<uint32_t>(grid.data[i]) : 0U;
  }
  return counts;
}

bool haveMatchingGridGeometry(
  const nav_msgs::msg::OccupancyGrid & lhs,
  const nav_msgs::msg::OccupancyGrid & rhs)
{
  return lhs.info.width == rhs.info.width &&
         lhs.info.height == rhs.info.height &&
         std::abs(lhs.info.resolution - rhs.info.resolution) <= 1.0e-6F &&
         std::abs(lhs.info.origin.position.x - rhs.info.origin.position.x) <= 1.0e-6 &&
         std::abs(lhs.info.origin.position.y - rhs.info.origin.position.y) <= 1.0e-6 &&
         lhs.header.frame_id == rhs.header.frame_id;
}

approach_map::XYPoint cellCenter(const approach_map::GridMeta & meta, std::size_t index)
{
  const std::size_t x_cell = index % meta.width_cells;
  const std::size_t y_cell = index / meta.width_cells;

  return approach_map::XYPoint{
    meta.origin_x_m + (static_cast<double>(x_cell) + 0.5) * meta.resolution_m,
    meta.origin_y_m + (static_cast<double>(y_cell) + 0.5) * meta.resolution_m};
}

std::optional<std::size_t> findLowestCostCellIndex(const approach_map::GridDataF32 & layer)
{
  std::optional<std::size_t> best_index;
  float best_cost = 0.0F;

  for (std::size_t i = 0; i < layer.values.size(); ++i) {
    const float value = layer.values[i];
    if (value < 0.0F || !std::isfinite(value)) {
      continue;
    }

    if (!best_index.has_value() || value < best_cost) {
      best_index = i;
      best_cost = value;
    }
  }

  return best_index;
}

std::size_t countFeasibleCells(const nav_msgs::msg::OccupancyGrid & grid)
{
  return static_cast<std::size_t>(std::count_if(
    grid.data.begin(), grid.data.end(), [](int8_t value) {return value > 0;}));
}

std::size_t countValidCostCells(const nav_msgs::msg::OccupancyGrid & grid)
{
  return static_cast<std::size_t>(std::count_if(
    grid.data.begin(), grid.data.end(), [](int8_t value) {return value >= 0;}));
}

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

std::string zeroPaddedNumber(uint64_t value, int width)
{
  std::ostringstream stream;
  stream << std::setw(width) << std::setfill('0') << value;
  return stream.str();
}

bool writeGridPgm(
  const std::filesystem::path & path,
  const nav_msgs::msg::OccupancyGrid & grid)
{
  std::ofstream stream(path, std::ios::binary);
  if (!stream.is_open()) {
    return false;
  }

  const std::size_t expected_size =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  if (grid.data.size() != expected_size) {
    return false;
  }

  stream << "P5\n" << grid.info.width << " " << grid.info.height << "\n255\n";
  for (std::size_t y = grid.info.height; y > 0U; --y) {
    const std::size_t row = y - 1U;
    for (std::size_t x = 0; x < grid.info.width; ++x) {
      const std::size_t index = row * grid.info.width + x;
      const int value = static_cast<int>(grid.data[index]);
      const unsigned char pixel = value < 0 ?
        static_cast<unsigned char>(127) :
        static_cast<unsigned char>(std::clamp(value, 0, 100) * 255 / 100);
      stream.write(reinterpret_cast<const char *>(&pixel), 1);
    }
  }

  return stream.good();
}

struct GraspReachResult
{
  std::vector<uint8_t> reach_mask;
  uint8_t status;
  uint32_t reachable_count;
};

GraspReachResult computeGraspReach(
  const approach_map::GridMeta & meta,
  const std::vector<approach_map::XYPoint> & grasp_objects,
  const std::vector<uint8_t> & feasible_mask,
  double radius_m)
{
  using StatusMsg = inha_interfaces::msg::GraspApproachStatus;

  const std::size_t num_cells = meta.width_cells * meta.height_cells;
  GraspReachResult result;
  result.reach_mask.assign(num_cells, 0U);
  result.status = StatusMsg::OK;
  result.reachable_count = 0U;

  const double r2 = radius_m * radius_m;
  uint32_t intersection_cells = 0U;
  uint32_t best_count = 0U;

  for (std::size_t i = 0; i < num_cells; ++i) {
    if (feasible_mask[i] == 0U) {
      continue;
    }
    const auto p = cellCenter(meta, i);
    uint32_t hit = 0U;
    for (const auto & obj : grasp_objects) {
      const double dx = p.x_m - obj.x_m;
      const double dy = p.y_m - obj.y_m;
      if (dx * dx + dy * dy <= r2) {
        ++hit;
      }
    }
    if (hit > best_count) {
      best_count = hit;
    }
    if (hit == grasp_objects.size()) {
      result.reach_mask[i] = 1U;
      ++intersection_cells;
    }
  }

  result.reachable_count = best_count;
  if (intersection_cells > 0U) {
    result.status = StatusMsg::OK;
  } else if (best_count > 0U) {
    result.status = StatusMsg::NO_INTERSECTION;
  } else {
    result.status = StatusMsg::NO_FEASIBLE_IN_INTERSECTION;
  }
  return result;
}

geometry_msgs::msg::PoseStamped makeBestCostPose(
  const std_msgs::msg::Header & header,
  const approach_map::XYPoint & from_point,
  const approach_map::XYPoint & to_point)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.pose.position.x = from_point.x_m;
  pose.pose.position.y = from_point.y_m;
  pose.pose.position.z = 0.0;

  const double yaw = std::atan2(
    to_point.y_m - from_point.y_m,
    to_point.x_m - from_point.x_m);
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);

  return pose;
}

class ApproachCostRunnerNode : public rclcpp::Node
{
public:
  ApproachCostRunnerNode()
  : Node("approach_cost_runner_node")
  {
    const std::string cost_config_path =
      this->declare_parameter<std::string>("cost_config_path", "");
    if (cost_config_path.empty()) {
      throw std::runtime_error("cost_config_path parameter is required");
    }

    cost_config_ = approach_cost::loadFinalCostConfigFromYaml(cost_config_path);
    runner_config_ = loadCostRunnerConfig(*this);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    final_cost_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      runner_config_.output_topic, 1);
    best_arrow_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      runner_config_.arrow_topic, 1);
    grasp_status_pub_ = this->create_publisher<inha_interfaces::msg::GraspApproachStatus>(
      runner_config_.grasp_status_topic, 1);

    auto target_point_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    target_point_qos.reliable();
    target_point_qos.transient_local();
    target_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      runner_config_.target_point_topic, target_point_qos,
      std::bind(&ApproachCostRunnerNode::targetPointCallback, this, std::placeholders::_1));

    auto grasp_targets_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    grasp_targets_qos.reliable();
    grasp_targets_qos.transient_local();
    grasp_targets_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      runner_config_.grasp_targets_topic, grasp_targets_qos,
      std::bind(&ApproachCostRunnerNode::graspTargetsCallback, this, std::placeholders::_1));

    feasible_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      runner_config_.feasible_map_topic, 10,
      std::bind(&ApproachCostRunnerNode::feasibleMapCallback, this, std::placeholders::_1));

    transition_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      runner_config_.transition_map_topic, 10,
      std::bind(&ApproachCostRunnerNode::transitionMapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Loaded cost config: %s", cost_config_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Loaded cost runner configuration from ROS parameters.");
    if (runner_config_.debug_save_enable) {
      RCLCPP_INFO(
        this->get_logger(),
        "Debug snapshots enabled: output=%s every_n_frames=%d max_frames_per_session=%d",
        runner_config_.debug_output_dir.c_str(),
        runner_config_.debug_save_every_n_frames,
        runner_config_.debug_max_frames_per_session);
    }
  }

private:
  tf2::Duration transformTimeout() const
  {
    return tf2::durationFromSec(std::max(0.0, runner_config_.transform_timeout_sec));
  }

  bool transformPoint(
    const geometry_msgs::msg::PointStamped & input,
    const std::string & target_frame,
    geometry_msgs::msg::PointStamped & output,
    const char * point_name,
    bool use_latest_transform = false)
  {
    if (input.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Cannot transform %s because the source frame is empty.", point_name);
      return false;
    }

    if (input.header.frame_id == target_frame) {
      output = input;
      return true;
    }

    try {
      auto transform_input = input;
      if (use_latest_transform) {
        transform_input.header.stamp.sec = 0;
        transform_input.header.stamp.nanosec = 0;
      }

      output = tf_buffer_->transform(transform_input, target_frame, transformTimeout());
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Failed to transform %s to %s: %s",
        point_name, target_frame.c_str(), ex.what());
      return false;
    }
  }

  bool lookupRobotPoint(
    const std::string & target_frame,
    const builtin_interfaces::msg::Time & stamp,
    approach_map::XYPoint & output)
  {
    geometry_msgs::msg::PointStamped robot_origin;
    robot_origin.header.frame_id = runner_config_.robot_frame_id;
    robot_origin.header.stamp = stamp;
    robot_origin.point.x = 0.0;
    robot_origin.point.y = 0.0;
    robot_origin.point.z = 0.0;

    geometry_msgs::msg::PointStamped transformed_point;
    if (!transformPoint(robot_origin, target_frame, transformed_point, "robot origin")) {
      return false;
    }

    output = approach_map::XYPoint{
      transformed_point.point.x,
      transformed_point.point.y};
    return true;
  }

  void startDebugSession(const geometry_msgs::msg::PointStamped & target)
  {
    if (!runner_config_.debug_save_enable) {
      return;
    }

    debug_csv_.close();
    debug_session_ready_ = false;
    debug_frame_index_ = 0U;
    debug_saved_snapshot_count_ = 0U;
    ++debug_session_index_;

    const auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const std::string directory_name =
      "session_" + zeroPaddedNumber(debug_session_index_, 4) + "_" +
      std::to_string(epoch_ms);
    debug_session_dir_ = std::filesystem::path(runner_config_.debug_output_dir) / directory_name;

    try {
      std::filesystem::create_directories(debug_session_dir_);

      debug_csv_.open(debug_session_dir_ / "frames.csv", std::ios::out);
      if (!debug_csv_.is_open()) {
        throw std::runtime_error("failed to open frames.csv");
      }

      debug_csv_ <<
        "frame,received_time_sec,map_stamp_sec,map_lag_ms,processing_ms,snapshot_write_ms,"
        "feasible_cells,valid_cost_cells,best_cost,best_x_m,best_y_m,"
        "target_x_m,target_y_m,target_distance_m,robot_x_m,robot_y_m,"
        "robot_distance_m,snapshot_saved\n";
      debug_csv_.flush();

      std::ofstream readme(debug_session_dir_ / "README.txt", std::ios::out);
      readme <<
        "Each frame_NNNNNN_feasible.pgm and frame_NNNNNN_final_cost.pgm pair "
        "belongs to the matching frames.csv row.\n"
        "PGM encoding: unknown=127, known grid value 0..100 mapped to 0..255.\n"
        "Lower final-cost brightness is preferred. Feasible cells are white.\n"
        "target_frame=" << target.header.frame_id << "\n"
        "target_x_m=" << target.point.x << "\n"
        "target_y_m=" << target.point.y << "\n";

      debug_session_ready_ = true;
      RCLCPP_INFO(
        this->get_logger(), "Started debug snapshot session: %s",
        debug_session_dir_.string().c_str());
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to start debug snapshot session in %s: %s",
        debug_session_dir_.string().c_str(), ex.what());
    }
  }

  void saveDebugFrame(
    const nav_msgs::msg::OccupancyGrid & feasible_grid,
    const nav_msgs::msg::OccupancyGrid & final_cost_grid,
    const approach_map::XYPoint & robot_point,
    const approach_map::XYPoint & target_point,
    const std::optional<std::size_t> & best_index,
    const std::chrono::steady_clock::time_point & callback_started,
    double callback_received_time_sec)
  {
    if (!runner_config_.debug_save_enable || !debug_session_ready_) {
      return;
    }

    ++debug_frame_index_;
    const bool snapshot_requested =
      debug_saved_snapshot_count_ <
      static_cast<uint64_t>(runner_config_.debug_max_frames_per_session) &&
      (debug_frame_index_ - 1U) %
      static_cast<uint64_t>(runner_config_.debug_save_every_n_frames) == 0U;

    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_started).count();

    bool snapshot_saved = false;
    double snapshot_write_ms = 0.0;
    if (snapshot_requested) {
      const auto snapshot_started = std::chrono::steady_clock::now();
      const std::string prefix = "frame_" + zeroPaddedNumber(debug_frame_index_, 6);
      const bool feasible_saved = writeGridPgm(
        debug_session_dir_ / (prefix + "_feasible.pgm"), feasible_grid);
      const bool final_cost_saved = writeGridPgm(
        debug_session_dir_ / (prefix + "_final_cost.pgm"), final_cost_grid);
      snapshot_write_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - snapshot_started).count();
      snapshot_saved = feasible_saved && final_cost_saved;
      if (snapshot_saved) {
        ++debug_saved_snapshot_count_;
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Failed to save one or more debug snapshot files in %s.",
          debug_session_dir_.string().c_str());
      }
    }

    const double map_stamp_sec = stampSeconds(feasible_grid.header.stamp);
    const double map_lag_ms = 1000.0 * (callback_received_time_sec - map_stamp_sec);

    double best_cost = -1.0;
    double best_x_m = 0.0;
    double best_y_m = 0.0;
    double target_distance_m = -1.0;
    double robot_distance_m = -1.0;
    if (best_index.has_value()) {
      const auto best_point = cellCenter(metaFromOccupancyGrid(final_cost_grid), best_index.value());
      best_cost = static_cast<double>(final_cost_grid.data[best_index.value()]) / 100.0;
      best_x_m = best_point.x_m;
      best_y_m = best_point.y_m;
      target_distance_m = std::hypot(
        best_point.x_m - target_point.x_m,
        best_point.y_m - target_point.y_m);
      robot_distance_m = std::hypot(
        best_point.x_m - robot_point.x_m,
        best_point.y_m - robot_point.y_m);
    }

    debug_csv_ << std::fixed << std::setprecision(6) <<
      debug_frame_index_ << "," <<
      callback_received_time_sec << "," <<
      map_stamp_sec << "," <<
      map_lag_ms << "," <<
      processing_ms << "," <<
      snapshot_write_ms << "," <<
      countFeasibleCells(feasible_grid) << "," <<
      countValidCostCells(final_cost_grid) << "," <<
      best_cost << "," <<
      best_x_m << "," <<
      best_y_m << "," <<
      target_point.x_m << "," <<
      target_point.y_m << "," <<
      target_distance_m << "," <<
      robot_point.x_m << "," <<
      robot_point.y_m << "," <<
      robot_distance_m << "," <<
      (snapshot_saved ? 1 : 0) << "\n";
    debug_csv_.flush();
  }

  void targetPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    latest_target_point_ = *msg;
    startDebugSession(*msg);
    RCLCPP_INFO(
      this->get_logger(), "Updated cost target point from topic %s",
      runner_config_.target_point_topic.c_str());
  }

  void transitionMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_transition_map_ = *msg;
  }

  void graspTargetsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    latest_grasp_targets_ = *msg;
    RCLCPP_INFO(
      this->get_logger(), "Updated grasp targets (%zu objects) from %s",
      msg->poses.size(), runner_config_.grasp_targets_topic.c_str());
  }

  std::vector<approach_map::XYPoint> graspObjectsInFrame(const std::string & target_frame)
  {
    std::vector<approach_map::XYPoint> objects;
    if (!latest_grasp_targets_.has_value()) {
      return objects;
    }
    const auto & arr = latest_grasp_targets_.value();
    objects.reserve(arr.poses.size());
    for (const auto & pose : arr.poses) {
      geometry_msgs::msg::PointStamped in;
      in.header = arr.header;
      in.point = pose.position;
      geometry_msgs::msg::PointStamped out;
      if (!transformPoint(in, target_frame, out, "grasp object", true)) {
        return {};
      }
      objects.push_back(approach_map::XYPoint{out.point.x, out.point.y});
    }
    return objects;
  }

  void publishGraspStatus(
    const std_msgs::msg::Header & header,
    uint8_t status,
    uint32_t num_objects,
    uint32_t reachable_count)
  {
    inha_interfaces::msg::GraspApproachStatus msg;
    msg.header = header;
    msg.status = status;
    msg.num_objects = num_objects;
    msg.reachable_count = reachable_count;
    grasp_status_pub_->publish(msg);
  }

  void feasibleMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const auto callback_started = std::chrono::steady_clock::now();
    const double callback_received_time_sec = this->now().seconds();

    if (!latest_target_point_.has_value()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for target point on %s before publishing cost map.",
        runner_config_.target_point_topic.c_str());
      return;
    }

    const std::string map_frame = msg->header.frame_id;
    if (map_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Received feasible map without frame_id.");
      return;
    }

    geometry_msgs::msg::PointStamped transformed_target;
    if (!transformPoint(
        latest_target_point_.value(), map_frame, transformed_target, "target point", true))
    {
      return;
    }

    approach_map::XYPoint robot_point;
    if (!lookupRobotPoint(map_frame, msg->header.stamp, robot_point)) {
      return;
    }

    approach_cost::CommonCostInput input;
    input.meta = metaFromOccupancyGrid(*msg);
    input.target_point_m = approach_map::XYPoint{
      transformed_target.point.x,
      transformed_target.point.y};
    input.robot_point_m = robot_point;
    input.candidate_mask = candidateMaskFromFeasibleMap(*msg);

    const auto grasp_objects = graspObjectsInFrame(map_frame);
    if (!grasp_objects.empty()) {
      const auto reach = computeGraspReach(
        input.meta, grasp_objects, input.candidate_mask, runner_config_.grasp_radius_m);
      for (std::size_t i = 0; i < input.candidate_mask.size(); ++i) {
        input.candidate_mask[i] = (input.candidate_mask[i] && reach.reach_mask[i]) ? 1U : 0U;
      }
      publishGraspStatus(
        msg->header, reach.status,
        static_cast<uint32_t>(grasp_objects.size()), reach.reachable_count);
    }

    if (latest_transition_map_.has_value()) {
      if (haveMatchingGridGeometry(*msg, latest_transition_map_.value())) {
        input.state_transition_counts = transitionCountsFromGrid(latest_transition_map_.value());
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Ignoring transition map because its grid geometry does not match feasible_map.");
      }
    }

    const auto common_layers = approach_cost::computeCostCommon(input, cost_config_.common);
    const auto final_cost_layer = approach_cost::computeFinalCost(common_layers, cost_config_);

    const auto final_cost_grid = toCostGrid(final_cost_layer, msg->header);
    final_cost_pub_->publish(final_cost_grid);

    const auto best_index = findLowestCostCellIndex(final_cost_layer);
    saveDebugFrame(
      *msg, final_cost_grid, robot_point, input.target_point_m, best_index, callback_started,
      callback_received_time_sec);
    if (!best_index.has_value()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "No valid cell found in final_cost_map for best cost pose.");
      return;
    }

    const auto best_point = cellCenter(final_cost_layer.meta, best_index.value());
    const auto target_point = input.target_point_m;

    best_arrow_pub_->publish(makeBestCostPose(
      msg->header, best_point, target_point));
  }

  approach_cost::FinalCostConfig cost_config_{};
  CostRunnerConfig runner_config_{};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::optional<geometry_msgs::msg::PointStamped> latest_target_point_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_transition_map_;
  std::optional<geometry_msgs::msg::PoseArray> latest_grasp_targets_;
  std::filesystem::path debug_session_dir_;
  std::ofstream debug_csv_;
  bool debug_session_ready_{false};
  uint64_t debug_session_index_{0U};
  uint64_t debug_frame_index_{0U};
  uint64_t debug_saved_snapshot_count_{0U};

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_point_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr feasible_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr transition_map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr grasp_targets_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr final_cost_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr best_arrow_pub_;
  rclcpp::Publisher<inha_interfaces::msg::GraspApproachStatus>::SharedPtr grasp_status_pub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachCostRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
