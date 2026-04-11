#include "approach_planning/config_io.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace approach_planning
{

namespace
{

template<typename T>
T readRequired(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required planner config key: ") + key);
  }
  return node[key].as<T>();
}

template<typename T>
T readOptional(const YAML::Node & node, const char * key, const T & default_value)
{
  return node[key] ? node[key].as<T>() : default_value;
}

}  // namespace

PlannerConfig loadPlannerConfigFromYaml(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const YAML::Node planner = root["planner"] ? root["planner"] : root;

  PlannerConfig config;
  config.weight_goal_cost = readRequired<double>(planner, "weight_goal_cost");
  config.weight_path_length = readRequired<double>(planner, "weight_path_length");
  config.weight_clearance_penalty = readRequired<double>(planner, "weight_clearance_penalty");
  config.clearance_penalty_power = readRequired<double>(planner, "clearance_penalty_power");
  config.minimum_clearance_ratio = readRequired<double>(planner, "minimum_clearance_ratio");
  config.allow_diagonal = readRequired<bool>(planner, "allow_diagonal");
  config.nearest_start_max_radius_cells =
    readRequired<std::size_t>(planner, "nearest_start_max_radius_cells");
  config.minimum_goal_distance_m =
    readOptional<double>(planner, "minimum_goal_distance_m", config.minimum_goal_distance_m);
  config.invalid_value = readOptional<float>(planner, "invalid_value", config.invalid_value);
  return config;
}

}  // namespace approach_planning
