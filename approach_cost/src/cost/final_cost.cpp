#include "approach_cost/cost/final_cost.hpp"

#include <stdexcept>

namespace approach_cost
{

approach_map::GridDataF32 computeFinalCost(
  const CommonCostLayers & common_layers,
  const FinalCostConfig & config)
{
  switch (config.mode) {
    case ModeId::Mode1:
      return common_layers.cost_common;
    default:
      throw std::runtime_error("Unsupported final cost mode");
  }
}

}  // namespace approach_cost
