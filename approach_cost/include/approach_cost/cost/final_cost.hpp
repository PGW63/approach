#pragma once

#include "approach_cost/cost/types.hpp"

namespace approach_cost
{

/**
 * @brief Compute the final cost layer for the selected mode.
 * @param common_layers Output produced by cost_common.
 * @param config Final cost composition configuration.
 * @return Final normalized cost layer.
 */
approach_map::GridDataF32 computeFinalCost(
  const CommonCostLayers & common_layers,
  const FinalCostConfig & config);

}  // namespace approach_cost
