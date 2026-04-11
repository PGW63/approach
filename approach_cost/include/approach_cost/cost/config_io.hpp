#pragma once

#include <string>

#include "approach_cost/cost/types.hpp"

namespace approach_cost
{

/**
 * @brief Load final cost configuration from a YAML file.
 * @param yaml_path Absolute or relative path to the cost config YAML file.
 * @return Parsed final cost configuration.
 */
FinalCostConfig loadFinalCostConfigFromYaml(const std::string & yaml_path);

}  // namespace approach_cost
