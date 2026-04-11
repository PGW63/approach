#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "approach_map/map/types.hpp"
#include "approach_planning/config_io.hpp"
#include "approach_planning/planner.hpp"

namespace
{

struct PlannerRunnerConfig
{
  std::string feasible_map_topic{"feasible_map"};
  std::string clearance_map_topic{"clearance_map"};
  std::string final_cost_map_topic{"final_cost_map"};
  std::string robot_frame_id{"base"};
  std::string planning_score_topic{"planning_score_map"};
  std::string path_topic{"planned_path"};
  std::string goal_topic{"best_planning_goal"};
  double transform_timeout_sec{0.1};
};

template<typename T>
T readRequired(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required planner runner config key: ") + key);
  }
  return node[key].as<T>();
}

PlannerRunnerConfig loadPlannerRunnerConfig(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const YAML::Node runner = root["planner_runner"] ? root["planner_runner"] : root;

  PlannerRunnerConfig config;
  config.feasible_map_topic = readRequired<std::string>(runner, "feasible_map_topic");
  config.clearance_map_topic = readRequired<std::string>(runner, "clearance_map_topic");
  config.final_cost_map_topic = readRequired<std::string>(runner, "final_cost_map_topic");
  config.robot_frame_id = readRequired<std::string>(runner, "robot_frame_id");
  config.planning_score_topic = readRequired<std::string>(runner, "planning_score_topic");
  config.path_topic = readRequired<std::string>(runner, "path_topic");
  config.goal_topic = readRequired<std::string>(runner, "goal_topic");
  config.transform_timeout_sec = readRequired<double>(runner, "transform_timeout_sec");
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

nav_msgs::msg::OccupancyGrid toScoreGrid(
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

std::vector<float> normalizedValuesFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid & grid)
{
  std::vector<float> values(grid.data.size(), -1.0F);
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    if (grid.data[i] < 0) {
      continue;
    }
    values[i] = std::clamp(static_cast<float>(grid.data[i]) / 100.0F, 0.0F, 1.0F);
  }
  return values;
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

nav_msgs::msg::Path makePathMessage(
  const approach_planning::PlanningResult & result,
  const approach_map::GridMeta & meta,
  const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;

  for (const std::size_t index : result.path_indices) {
    const auto center = cellCenter(meta, index);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = center.x_m;
    pose.pose.position.y = center.y_m;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

class ApproachPlanningRunnerNode : public rclcpp::Node
{
public:
  ApproachPlanningRunnerNode()
  : Node("approach_planning_runner_node")
  {
    const auto runner_share = ament_index_cpp::get_package_share_directory("approach_map_runner");
    const std::string config_path = runner_share + "/config/planner_runner_config.yaml";

    planner_config_ = approach_planning::loadPlannerConfigFromYaml(config_path);
    runner_config_ = loadPlannerRunnerConfig(config_path);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    feasible_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      runner_config_.feasible_map_topic, 10,
      std::bind(&ApproachPlanningRunnerNode::feasibleMapCallback, this, std::placeholders::_1));
    clearance_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      runner_config_.clearance_map_topic, 10,
      std::bind(&ApproachPlanningRunnerNode::clearanceMapCallback, this, std::placeholders::_1));
    final_cost_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      runner_config_.final_cost_map_topic, 10,
      std::bind(&ApproachPlanningRunnerNode::finalCostMapCallback, this, std::placeholders::_1));

    planning_score_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      runner_config_.planning_score_topic, 1);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(runner_config_.path_topic, 1);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(runner_config_.goal_topic, 1);

    RCLCPP_INFO(this->get_logger(), "Loaded planning runner config: %s", config_path.c_str());
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
    const char * point_name)
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
      output = tf_buffer_->transform(input, target_frame, transformTimeout());
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

  void feasibleMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_feasible_map_ = *msg;
    tryPlan();
  }

  void clearanceMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_clearance_map_ = *msg;
    tryPlan();
  }

  void finalCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_final_cost_map_ = *msg;
    tryPlan();
  }

  void tryPlan()
  {
    if (
      !latest_feasible_map_.has_value() ||
      !latest_clearance_map_.has_value() ||
      !latest_final_cost_map_.has_value())
    {
      return;
    }

    const auto & feasible_map = latest_feasible_map_.value();
    const auto & clearance_map = latest_clearance_map_.value();
    const auto & final_cost_map = latest_final_cost_map_.value();

    if (!haveMatchingGridGeometry(final_cost_map, feasible_map)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Ignoring planner update because feasible_map geometry does not match final_cost_map.");
      return;
    }

    if (!haveMatchingGridGeometry(final_cost_map, clearance_map)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Ignoring planner update because clearance_map geometry does not match final_cost_map.");
      return;
    }

    if (final_cost_map.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Received final_cost_map without frame_id.");
      return;
    }

    approach_map::XYPoint robot_point;
    if (!lookupRobotPoint(final_cost_map.header.frame_id, final_cost_map.header.stamp, robot_point)) {
      return;
    }

    approach_planning::PlanningInput input;
    input.meta = metaFromOccupancyGrid(final_cost_map);
    input.robot_point_m = robot_point;
    input.traversable_mask = candidateMaskFromFeasibleMap(feasible_map);
    input.goal_costs = normalizedValuesFromOccupancyGrid(final_cost_map);
    input.clearance_scores = normalizedValuesFromOccupancyGrid(clearance_map);

    const auto result = approach_planning::computePlan(input, planner_config_);
    planning_score_pub_->publish(toScoreGrid(result.planning_score_layer, final_cost_map.header));

    nav_msgs::msg::Path path_msg;
    path_msg.header = final_cost_map.header;
    if (result.valid) {
      path_msg = makePathMessage(result, input.meta, final_cost_map.header);
    }
    path_pub_->publish(path_msg);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Published planning path: valid=%s poses=%zu start_index=%zu goal_index=%zu "
      "goal_cost=%.3f path_cost=%.3f total_cost=%.3f",
      result.valid ? "true" : "false", path_msg.poses.size(), result.start_index, result.goal_index,
      result.goal_cost, result.path_cost, result.total_cost);

    if (!result.valid) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "No reachable goal found in feasible_map for the current planning update.");
      return;
    }

    const auto goal_point = cellCenter(input.meta, result.goal_index);
    geometry_msgs::msg::PointStamped goal_msg;
    goal_msg.header = final_cost_map.header;
    goal_msg.point.x = goal_point.x_m;
    goal_msg.point.y = goal_point.y_m;
    goal_msg.point.z = 0.0;
    goal_pub_->publish(goal_msg);
  }

  approach_planning::PlannerConfig planner_config_{};
  PlannerRunnerConfig runner_config_{};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_feasible_map_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_clearance_map_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_final_cost_map_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr feasible_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr clearance_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr final_cost_map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_score_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachPlanningRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
