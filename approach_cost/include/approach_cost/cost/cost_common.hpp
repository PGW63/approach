#pragma once

#include "approach_cost/cost/types.hpp"

namespace approach_cost
{

/**
 * @brief Compute normalized target-distance, stability, and approach-distance costs.
 * @param input Common cost inputs defined on the current map grid.
 * @param config Weighting and normalization configuration for common costs.
 * @return Struct containing each normalized cost layer and their weighted combination.
 */
CommonCostLayers computeCostCommon(const CommonCostInput & input, const CommonCostConfig & config);

}  // namespace approach_cost
