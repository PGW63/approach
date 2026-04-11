#include "approach_map/map/config_io.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace approach_map
{

namespace
{

template<typename T>
T readRequired(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required map config key: ") + key);
  }
  return node[key].as<T>();
}

}  // namespace

Config loadConfigFromYaml(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const YAML::Node map = root["map"] ? root["map"] : root;

  Config config;
  config.resolution_m = readRequired<double>(map, "resolution_m");
  config.width_m = readRequired<double>(map, "width_m");
  config.height_m = readRequired<double>(map, "height_m");
  config.obstacle_hit_cap_per_update = readRequired<int>(map, "obstacle_hit_cap_per_update");
  config.ground_hit_cap_per_update = readRequired<int>(map, "ground_hit_cap_per_update");
  config.obstacle_weight = readRequired<int>(map, "obstacle_weight");
  config.ground_weight = readRequired<int>(map, "ground_weight");
  config.evidence_clip_value = readRequired<int>(map, "evidence_clip_value");
  config.occupied_score_threshold = readRequired<int>(map, "occupied_score_threshold");
  config.free_score_threshold = readRequired<int>(map, "free_score_threshold");
  config.footprint_length_m = readRequired<double>(map, "footprint_length_m");
  config.footprint_width_m = readRequired<double>(map, "footprint_width_m");
  config.footprint_margin_m = readRequired<double>(map, "footprint_margin_m");
  config.heading_bin_count = readRequired<std::size_t>(map, "heading_bin_count");
  config.unknown_is_blocked_for_feasibility =
    readRequired<bool>(map, "unknown_is_blocked_for_feasibility");
  return config;
}

}  // namespace approach_map
