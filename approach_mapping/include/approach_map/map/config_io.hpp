#pragma once

#include <string>

#include "approach_map/map/types.hpp"

namespace approach_map
{

/**
 * @brief Load map builder configuration from a YAML file.
 * @param yaml_path Absolute or relative path to the map config YAML file.
 * @return Parsed mapping configuration.
 */
Config loadConfigFromYaml(const std::string & yaml_path);

}  // namespace approach_map
