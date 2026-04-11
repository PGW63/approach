#include "approach_planning/planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <utility>

namespace approach_planning
{

namespace
{

constexpr double kSqrt2 = 1.4142135623730951;

struct QueueNode
{
  double cost;
  std::size_t index;
};

struct QueueCompare
{
  bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
  {
    return lhs.cost > rhs.cost;
  }
};

approach_map::XYPoint cellCenter(const approach_map::GridMeta & meta, std::size_t index)
{
  const std::size_t x_cell = index % meta.width_cells;
  const std::size_t y_cell = index / meta.width_cells;

  return approach_map::XYPoint{
    meta.origin_x_m + (static_cast<double>(x_cell) + 0.5) * meta.resolution_m,
    meta.origin_y_m + (static_cast<double>(y_cell) + 0.5) * meta.resolution_m};
}

bool isValidNormalizedValue(float value)
{
  return value >= 0.0F && std::isfinite(value);
}

bool isTraversableCell(
  const PlanningInput & input,
  const PlannerConfig & config,
  std::size_t index)
{
  if (index >= input.traversable_mask.size() || input.traversable_mask[index] == 0U) {
    return false;
  }

  if (!input.clearance_scores.empty()) {
    if (index >= input.clearance_scores.size()) {
      return false;
    }

    if (!isValidNormalizedValue(input.clearance_scores[index])) {
      return false;
    }

    if (static_cast<double>(input.clearance_scores[index]) + 1.0e-6 <
      config.minimum_clearance_ratio)
    {
      return false;
    }
  }

  return true;
}

std::optional<std::size_t> findNearestTraversableStartIndex(
  const PlanningInput & input,
  const PlannerConfig & config)
{
  const auto exact_index = approach_map::worldToIndex(
    input.meta, input.robot_point_m.x_m, input.robot_point_m.y_m);
  if (exact_index.has_value() && isTraversableCell(input, config, exact_index.value())) {
    return exact_index;
  }

  const double max_radius_m =
    static_cast<double>(config.nearest_start_max_radius_cells) * input.meta.resolution_m;
  const bool use_radius_limit = config.nearest_start_max_radius_cells > 0U;

  std::optional<std::size_t> best_index;
  double best_distance_sq = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < input.traversable_mask.size(); ++i) {
    if (!isTraversableCell(input, config, i)) {
      continue;
    }

    const auto center = cellCenter(input.meta, i);
    const double dx = center.x_m - input.robot_point_m.x_m;
    const double dy = center.y_m - input.robot_point_m.y_m;
    const double distance_sq = dx * dx + dy * dy;

    if (use_radius_limit && distance_sq > max_radius_m * max_radius_m) {
      continue;
    }

    if (!best_index.has_value() || distance_sq < best_distance_sq) {
      best_index = i;
      best_distance_sq = distance_sq;
    }
  }

  return best_index;
}

approach_map::GridDataF32 normalizeLayer(
  const approach_map::GridMeta & meta,
  const std::vector<float> & raw_values,
  float invalid_value)
{
  std::vector<float> normalized(raw_values.size(), invalid_value);
  float min_value = std::numeric_limits<float>::infinity();
  float max_value = -std::numeric_limits<float>::infinity();

  for (const float value : raw_values) {
    if (!isValidNormalizedValue(value)) {
      continue;
    }
    min_value = std::min(min_value, value);
    max_value = std::max(max_value, value);
  }

  if (!std::isfinite(min_value) || !std::isfinite(max_value)) {
    return approach_map::makeGridDataF32(meta, std::move(normalized));
  }

  const float span = max_value - min_value;
  for (std::size_t i = 0; i < raw_values.size(); ++i) {
    if (!isValidNormalizedValue(raw_values[i])) {
      continue;
    }

    normalized[i] = span <= 1.0e-6F ? 0.0F : (raw_values[i] - min_value) / span;
  }

  return approach_map::makeGridDataF32(meta, std::move(normalized));
}

}  // namespace

PlanningResult computePlan(const PlanningInput & input, const PlannerConfig & config)
{
  const std::size_t expected_cell_count = approach_map::cellCount(input.meta);
  if (input.traversable_mask.size() != expected_cell_count) {
    throw std::runtime_error("traversable_mask size does not match grid cell count");
  }
  if (input.goal_costs.size() != expected_cell_count) {
    throw std::runtime_error("goal_costs size does not match grid cell count");
  }
  if (!input.clearance_scores.empty() && input.clearance_scores.size() != expected_cell_count) {
    throw std::runtime_error("clearance_scores size does not match grid cell count");
  }

  PlanningResult result;
  result.planning_score_layer = approach_map::makeGridDataF32(
    input.meta, std::vector<float>(expected_cell_count, config.invalid_value));

  const auto maybe_start_index = findNearestTraversableStartIndex(input, config);
  if (!maybe_start_index.has_value()) {
    return result;
  }

  result.start_index = maybe_start_index.value();

  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> path_costs(expected_cell_count, inf);
  std::vector<std::size_t> predecessors(expected_cell_count, expected_cell_count);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> queue;

  path_costs[result.start_index] = 0.0;
  queue.push({0.0, result.start_index});

  const std::array<std::pair<int, int>, 8> deltas{
    std::make_pair(-1, 0),
    std::make_pair(1, 0),
    std::make_pair(0, -1),
    std::make_pair(0, 1),
    std::make_pair(-1, -1),
    std::make_pair(-1, 1),
    std::make_pair(1, -1),
    std::make_pair(1, 1)};

  while (!queue.empty()) {
    const QueueNode current = queue.top();
    queue.pop();

    if (current.cost > path_costs[current.index]) {
      continue;
    }

    const std::size_t x_cell = current.index % input.meta.width_cells;
    const std::size_t y_cell = current.index / input.meta.width_cells;

    for (const auto & delta : deltas) {
      const bool diagonal = delta.first != 0 && delta.second != 0;
      if (!config.allow_diagonal && diagonal) {
        continue;
      }

      const int nx = static_cast<int>(x_cell) + delta.first;
      const int ny = static_cast<int>(y_cell) + delta.second;
      if (!approach_map::isInsideGrid(input.meta, nx, ny)) {
        continue;
      }

      const std::size_t neighbor_index = approach_map::flattenIndex(
        input.meta, static_cast<std::size_t>(nx), static_cast<std::size_t>(ny));
      if (!isTraversableCell(input, config, neighbor_index)) {
        continue;
      }

      double clearance_ratio = 1.0;
      if (!input.clearance_scores.empty()) {
        clearance_ratio = std::clamp(
          static_cast<double>(input.clearance_scores[neighbor_index]), 0.0, 1.0);
      }

      const double step_length_m =
        input.meta.resolution_m * (diagonal ? kSqrt2 : 1.0);
      const double clearance_penalty = std::pow(
        std::max(0.0, 1.0 - clearance_ratio), config.clearance_penalty_power);
      const double edge_cost = step_length_m * (
        config.weight_path_length +
        config.weight_clearance_penalty * clearance_penalty);
      const double candidate_cost = current.cost + edge_cost;

      if (candidate_cost >= path_costs[neighbor_index]) {
        continue;
      }

      path_costs[neighbor_index] = candidate_cost;
      predecessors[neighbor_index] = current.index;
      queue.push({candidate_cost, neighbor_index});
    }
  }

  std::vector<float> raw_total_scores(expected_cell_count, config.invalid_value);
  std::optional<std::size_t> best_goal_index;
  double best_goal_cost = inf;
  double best_goal_path_cost = inf;
  double best_total_cost = inf;
  const auto start_center = cellCenter(input.meta, result.start_index);
  const double minimum_goal_distance_m = std::max(0.0, config.minimum_goal_distance_m);

  for (std::size_t i = 0; i < expected_cell_count; ++i) {
    if (!std::isfinite(path_costs[i]) || !isValidNormalizedValue(input.goal_costs[i])) {
      continue;
    }

    if (minimum_goal_distance_m > 0.0) {
      const auto candidate_center = cellCenter(input.meta, i);
      const double distance_from_start_m = std::hypot(
        candidate_center.x_m - start_center.x_m,
        candidate_center.y_m - start_center.y_m);
      if (distance_from_start_m + 1.0e-6 < minimum_goal_distance_m) {
        continue;
      }
    }

    const double goal_cost = std::clamp(static_cast<double>(input.goal_costs[i]), 0.0, 1.0);
    const double total_cost = path_costs[i] + config.weight_goal_cost * goal_cost;
    raw_total_scores[i] = static_cast<float>(total_cost);

    // Keep the destination anchored to the cost map; path cost only breaks ties.
    if (
      !best_goal_index.has_value() ||
      goal_cost + 1.0e-6 < best_goal_cost ||
      (std::abs(goal_cost - best_goal_cost) <= 1.0e-6 && path_costs[i] < best_goal_path_cost))
    {
      best_goal_index = i;
      best_goal_cost = goal_cost;
      best_goal_path_cost = path_costs[i];
      best_total_cost = total_cost;
    }
  }

  result.planning_score_layer = normalizeLayer(input.meta, raw_total_scores, config.invalid_value);

  if (!best_goal_index.has_value()) {
    return result;
  }

  result.valid = true;
  result.goal_index = best_goal_index.value();
  result.goal_cost = std::clamp(static_cast<double>(input.goal_costs[result.goal_index]), 0.0, 1.0);
  result.path_cost = path_costs[result.goal_index];
  result.total_cost = best_total_cost;

  std::vector<std::size_t> reversed_path;
  std::size_t current_index = result.goal_index;
  reversed_path.push_back(current_index);

  while (current_index != result.start_index) {
    const std::size_t predecessor = predecessors[current_index];
    if (predecessor >= expected_cell_count) {
      result.valid = false;
      result.path_indices.clear();
      return result;
    }

    current_index = predecessor;
    reversed_path.push_back(current_index);
  }

  result.path_indices.assign(reversed_path.rbegin(), reversed_path.rend());
  return result;
}

}  // namespace approach_planning
