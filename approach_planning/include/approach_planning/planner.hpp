#pragma once

#include <cstddef>
#include <vector>

#include "approach_map/map/types.hpp"

namespace approach_planning
{

struct PlannerConfig
{
  double weight_goal_cost{3.0};
  double weight_path_length{1.0};
  double weight_clearance_penalty{2.0};
  double clearance_penalty_power{2.0};
  double minimum_clearance_ratio{0.0};
  bool allow_diagonal{true};
  std::size_t nearest_start_max_radius_cells{5};
  double minimum_goal_distance_m{0.0};
  float invalid_value{-1.0F};
};

struct PlanningInput
{
  approach_map::GridMeta meta;
  approach_map::XYPoint robot_point_m;
  std::vector<uint8_t> traversable_mask;
  std::vector<float> goal_costs;
  std::vector<float> clearance_scores;
};

struct PlanningResult
{
  bool valid{false};
  std::size_t start_index{0};
  std::size_t goal_index{0};
  double goal_cost{0.0};
  double path_cost{0.0};
  double total_cost{0.0};
  approach_map::GridDataF32 planning_score_layer;
  std::vector<std::size_t> path_indices;
};

PlanningResult computePlan(const PlanningInput & input, const PlannerConfig & config);

}  // namespace approach_planning
