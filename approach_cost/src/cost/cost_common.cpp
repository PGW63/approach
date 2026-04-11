#include "approach_cost/cost/cost_common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace approach_cost
{

namespace
{

bool isCandidate(const CommonCostInput & input, std::size_t index)
{
  if (input.candidate_mask.empty()) {
    return true;
  }
  if (index >= input.candidate_mask.size()) {
    return false;
  }
  return input.candidate_mask[index] != 0U;
}

approach_map::XYPoint cellCenter(const approach_map::GridMeta & meta, std::size_t index)
{
  const std::size_t x_cell = index % meta.width_cells;
  const std::size_t y_cell = index / meta.width_cells;

  return approach_map::XYPoint{
    meta.origin_x_m + (static_cast<double>(x_cell) + 0.5) * meta.resolution_m,
    meta.origin_y_m + (static_cast<double>(y_cell) + 0.5) * meta.resolution_m};
}

approach_map::GridDataF32 normalizeLayer(
  const approach_map::GridMeta & meta,
  const std::vector<float> & raw_values,
  const CommonCostInput & input,
  float invalid_value)
{
  std::vector<float> normalized(raw_values.size(), invalid_value);
  float min_value = std::numeric_limits<float>::infinity();
  float max_value = -std::numeric_limits<float>::infinity();

  for (std::size_t i = 0; i < raw_values.size(); ++i) {
    if (!isCandidate(input, i) || raw_values[i] < 0.0F || !std::isfinite(raw_values[i])) {
      continue;
    }

    min_value = std::min(min_value, raw_values[i]);
    max_value = std::max(max_value, raw_values[i]);
  }

  if (!std::isfinite(min_value) || !std::isfinite(max_value)) {
    return approach_map::makeGridDataF32(meta, std::move(normalized));
  }

  const float span = max_value - min_value;

  for (std::size_t i = 0; i < raw_values.size(); ++i) {
    if (!isCandidate(input, i) || raw_values[i] < 0.0F || !std::isfinite(raw_values[i])) {
      continue;
    }

    normalized[i] = span <= 1.0e-6F ? 0.0F : (raw_values[i] - min_value) / span;
  }

  return approach_map::makeGridDataF32(meta, std::move(normalized));
}

approach_map::GridDataF32 computeTargetDistanceCostInternal(
  const CommonCostInput & input,
  float invalid_value)
{
  std::vector<float> raw(approach_map::cellCount(input.meta), invalid_value);

  for (std::size_t i = 0; i < raw.size(); ++i) {
    if (!isCandidate(input, i)) {
      continue;
    }

    const approach_map::XYPoint center = cellCenter(input.meta, i);
    raw[i] = static_cast<float>(std::hypot(
      center.x_m - input.target_point_m.x_m,
      center.y_m - input.target_point_m.y_m));
  }

  return normalizeLayer(input.meta, raw, input, invalid_value);
}

approach_map::GridDataF32 computeApproachDistanceCostInternal(
  const CommonCostInput & input,
  float invalid_value)
{
  std::vector<float> raw(approach_map::cellCount(input.meta), invalid_value);

  for (std::size_t i = 0; i < raw.size(); ++i) {
    if (!isCandidate(input, i)) {
      continue;
    }

    const approach_map::XYPoint center = cellCenter(input.meta, i);
    raw[i] = static_cast<float>(std::hypot(
      center.x_m - input.robot_point_m.x_m,
      center.y_m - input.robot_point_m.y_m));
  }

  return normalizeLayer(input.meta, raw, input, invalid_value);
}

approach_map::GridDataF32 computeStabilityCostInternal(
  const CommonCostInput & input,
  const CommonCostConfig & config)
{
  std::vector<float> raw(approach_map::cellCount(input.meta), config.invalid_value);

  if (!input.state_transition_counts.empty() && input.state_transition_counts.size() != raw.size()) {
    throw std::runtime_error("state_transition_counts size does not match grid cell count");
  }

  for (std::size_t i = 0; i < raw.size(); ++i) {
    if (!isCandidate(input, i)) {
      continue;
    }

    const std::size_t x_cell = i % input.meta.width_cells;
    const std::size_t y_cell = i / input.meta.width_cells;

    double sum_transitions = 0.0;
    std::size_t sample_count = 0;

    for (int dy = -static_cast<int>(config.stability_window_radius_cells);
      dy <= static_cast<int>(config.stability_window_radius_cells); ++dy)
    {
      for (int dx = -static_cast<int>(config.stability_window_radius_cells);
        dx <= static_cast<int>(config.stability_window_radius_cells); ++dx)
      {
        const int nx = static_cast<int>(x_cell) + dx;
        const int ny = static_cast<int>(y_cell) + dy;
        if (!approach_map::isInsideGrid(input.meta, nx, ny)) {
          continue;
        }

        const std::size_t neighbor_index = approach_map::flattenIndex(
          input.meta, static_cast<std::size_t>(nx), static_cast<std::size_t>(ny));
        const double transitions = input.state_transition_counts.empty() ? 0.0 :
          static_cast<double>(input.state_transition_counts[neighbor_index]);
        sum_transitions += transitions;
        ++sample_count;
      }
    }

    raw[i] = sample_count == 0 ? 0.0F : static_cast<float>(sum_transitions / sample_count);
  }

  return normalizeLayer(input.meta, raw, input, config.invalid_value);
}

approach_map::GridDataF32 combineCommonCosts(
  const CommonCostLayers & layers,
  const CommonCostInput & input,
  const CommonCostConfig & config)
{
  std::vector<float> combined(approach_map::cellCount(input.meta), config.invalid_value);
  const double total_weight =
    config.weight_target_distance + config.weight_stability + config.weight_approach_distance;

  for (std::size_t i = 0; i < combined.size(); ++i) {
    if (!isCandidate(input, i)) {
      continue;
    }

    const float target_cost = layers.target_distance_cost.values[i];
    const float stability_cost = layers.stability_cost.values[i];
    const float approach_cost = layers.approach_distance_cost.values[i];

    if (target_cost < 0.0F || stability_cost < 0.0F || approach_cost < 0.0F) {
      continue;
    }

    if (total_weight <= 1.0e-9) {
      combined[i] = 0.0F;
      continue;
    }

    combined[i] = static_cast<float>(
      (config.weight_target_distance * target_cost +
       config.weight_stability * stability_cost +
       config.weight_approach_distance * approach_cost) / total_weight);
  }

  return approach_map::makeGridDataF32(input.meta, std::move(combined));
}

}  // namespace

CommonCostLayers computeCostCommon(const CommonCostInput & input, const CommonCostConfig & config)
{
  if (!input.candidate_mask.empty() && input.candidate_mask.size() != approach_map::cellCount(input.meta)) {
    throw std::runtime_error("candidate_mask size does not match grid cell count");
  }

  CommonCostLayers layers;
  layers.target_distance_cost = computeTargetDistanceCostInternal(input, config.invalid_value);
  layers.stability_cost = computeStabilityCostInternal(input, config);
  layers.approach_distance_cost = computeApproachDistanceCostInternal(input, config.invalid_value);
  layers.cost_common = combineCommonCosts(layers, input, config);
  return layers;
}

}  // namespace approach_cost
