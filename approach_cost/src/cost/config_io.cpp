#include "approach_cost/cost/config_io.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace approach_cost
{

namespace
{

template<typename T>
T readRequired(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required cost config key: ") + key);
  }
  return node[key].as<T>();
}

ModeId readMode(const YAML::Node & node)
{
  const int mode_value = node.as<int>();
  switch (mode_value) {
    case 1:
      return ModeId::Mode1;
    default:
      throw std::runtime_error("Unsupported cost mode: " + std::to_string(mode_value));
  }
}

}  // namespace

FinalCostConfig loadFinalCostConfigFromYaml(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const YAML::Node cost = root["cost"] ? root["cost"] : root;
  const YAML::Node common = cost["common"] ? cost["common"] : cost;

  FinalCostConfig config;
  config.mode = readMode(readRequired<YAML::Node>(cost, "mode"));
  config.common.weight_target_distance = readRequired<double>(common, "weight_target_distance");
  config.common.weight_stability = readRequired<double>(common, "weight_stability");
  config.common.weight_approach_distance = readRequired<double>(common, "weight_approach_distance");
  config.common.stability_window_radius_cells =
    readRequired<std::size_t>(common, "stability_window_radius_cells");
  config.common.invalid_value = readRequired<float>(common, "invalid_value");
  return config;
}

}  // namespace approach_cost
