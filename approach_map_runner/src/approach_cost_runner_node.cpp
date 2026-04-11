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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>

#include "approach_cost/cost.hpp"

namespace
{

struct CostRunnerConfig
{
  std::string feasible_map_topic{"feasible_map"};
  std::string transition_map_topic{"transition_count_map"};
  std::string target_point_topic{"/clicked_point"};
  std::string robot_frame_id{"base_link"};
  std::string output_topic{"final_cost_map"};
  std::string arrow_topic{"best_cost_arrow"};
  double transform_timeout_sec{0.1};
};

template<typename T>
T readRequired(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required cost runner config key: ") + key);
  }
  return node[key].as<T>();
}

template<typename T>
T readOptional(const YAML::Node & node, const char * key, const T & default_value)
{
  return node[key] ? node[key].as<T>() : default_value;
}

CostRunnerConfig loadCostRunnerConfig(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const YAML::Node runner = root["cost_runner"] ? root["cost_runner"] : root;

  CostRunnerConfig config;
  config.feasible_map_topic = readRequired<std::string>(runner, "feasible_map_topic");
  config.transition_map_topic = readRequired<std::string>(runner, "transition_map_topic");
  config.target_point_topic = readRequired<std::string>(runner, "target_point_topic");
  config.robot_frame_id = readRequired<std::string>(runner, "robot_frame_id");
  config.output_topic = readRequired<std::string>(runner, "output_topic");
  config.arrow_topic = readOptional<std::string>(runner, "arrow_topic", config.arrow_topic);
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

visualization_msgs::msg::Marker makeDeleteArrowMarker(const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "best_cost";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  return marker;
}

visualization_msgs::msg::Marker makeArrowMarker(
  const std_msgs::msg::Header & header,
  const approach_map::XYPoint & from_point,
  const approach_map::XYPoint & to_point,
  double resolution_m)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "best_cost";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = std::max(0.02, 0.25 * resolution_m);
  marker.scale.y = std::max(0.04, 0.50 * resolution_m);
  marker.scale.z = std::max(0.06, 0.75 * resolution_m);
  marker.color.r = 0.95F;
  marker.color.g = 0.25F;
  marker.color.b = 0.10F;
  marker.color.a = 0.95F;

  geometry_msgs::msg::Point start;
  start.x = from_point.x_m;
  start.y = from_point.y_m;
  start.z = 0.05;

  geometry_msgs::msg::Point end;
  end.x = to_point.x_m;
  end.y = to_point.y_m;
  end.z = 0.05;

  marker.points.push_back(start);
  marker.points.push_back(end);
  return marker;
}

class ApproachCostRunnerNode : public rclcpp::Node
{
public:
  ApproachCostRunnerNode()
  : Node("approach_cost_runner_node")
  {
    const auto cost_share = ament_index_cpp::get_package_share_directory("approach_cost");
    const auto runner_share = ament_index_cpp::get_package_share_directory("approach_map_runner");

    const std::string cost_config_path = cost_share + "/config/cost_config.yaml";
    const std::string runner_config_path = runner_share + "/config/cost_runner_config.yaml";

    cost_config_ = approach_cost::loadFinalCostConfigFromYaml(cost_config_path);
    runner_config_ = loadCostRunnerConfig(runner_config_path);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    final_cost_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      runner_config_.output_topic, 1);
    best_arrow_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      runner_config_.arrow_topic, 1);

    target_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      runner_config_.target_point_topic, 10,
      std::bind(&ApproachCostRunnerNode::targetPointCallback, this, std::placeholders::_1));

    feasible_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      runner_config_.feasible_map_topic, 10,
      std::bind(&ApproachCostRunnerNode::feasibleMapCallback, this, std::placeholders::_1));

    transition_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      runner_config_.transition_map_topic, 10,
      std::bind(&ApproachCostRunnerNode::transitionMapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Loaded cost config: %s", cost_config_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Loaded cost runner config: %s", runner_config_path.c_str());
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

  void targetPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    latest_target_point_ = *msg;
    RCLCPP_INFO(
      this->get_logger(), "Updated cost target point from topic %s",
      runner_config_.target_point_topic.c_str());
  }

  void transitionMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_transition_map_ = *msg;
  }

  void feasibleMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (!latest_target_point_.has_value()) {
      best_arrow_pub_->publish(makeDeleteArrowMarker(msg->header));
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
        latest_target_point_.value(), map_frame, transformed_target, "target point"))
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

    final_cost_pub_->publish(toCostGrid(final_cost_layer, msg->header));

    const auto best_index = findLowestCostCellIndex(final_cost_layer);
    if (!best_index.has_value()) {
      best_arrow_pub_->publish(makeDeleteArrowMarker(msg->header));
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "No valid cell found in final_cost_map for arrow visualization.");
      return;
    }

    const auto best_point = cellCenter(final_cost_layer.meta, best_index.value());
    const auto target_point = input.target_point_m;
    const double arrow_length_m = std::hypot(
      target_point.x_m - best_point.x_m,
      target_point.y_m - best_point.y_m);

    if (arrow_length_m <= 1.0e-6) {
      best_arrow_pub_->publish(makeDeleteArrowMarker(msg->header));
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Best-cost cell center matches the target point, so the arrow is hidden.");
      return;
    }

    best_arrow_pub_->publish(makeArrowMarker(
      msg->header, best_point, target_point, final_cost_layer.meta.resolution_m));
  }

  approach_cost::FinalCostConfig cost_config_{};
  CostRunnerConfig runner_config_{};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::optional<geometry_msgs::msg::PointStamped> latest_target_point_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_transition_map_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_point_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr feasible_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr transition_map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr final_cost_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr best_arrow_pub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachCostRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
