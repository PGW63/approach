#include "approach_map/map/builder.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace approach_map
{

namespace
{

constexpr double kTwoPi = 6.2831853071795862;

}  // namespace

Builder::Builder(const Config & config, const Origin & origin)
{
  reconfigure(config, origin);
}

void Builder::reconfigure(const Config & config, const Origin & origin)
{
  config_ = config;
  meta_ = makeGridMeta(config_, origin);
  resizeBuffers();
  buildFootprintStencils();
}

void Builder::setOrigin(const Origin & origin)
{
  meta_ = makeGridMeta(config_, origin);
  resizeBuffers();
  buildFootprintStencils();
}

void Builder::shiftOriginPreserveEvidence(const Origin & origin)
{
  const GridMeta old_meta = meta_;

  const auto old_evidence_scores = evidence_scores_;
  const auto old_state_transition_counts = state_transition_counts_;
  const auto old_observed = observed_;
  const auto old_cell_states = cell_states_;

  meta_ = makeGridMeta(config_, origin);

  const std::size_t new_cell_count = approach_map::cellCount(meta_);

  std::vector<int> new_evidence_scores(new_cell_count, 0);
  std::vector<uint32_t> new_state_transition_counts(new_cell_count, 0U);
  std::vector<uint8_t> new_observed(new_cell_count, 0U);
  std::vector<CellState> new_cell_states(new_cell_count, CellState::Unknown);

  for (std::size_t old_index = 0; old_index < old_observed.size(); ++old_index) {
    if (old_observed[old_index] == 0U) {
      continue;
    }

    const std::size_t old_x = old_index % old_meta.width_cells;
    const std::size_t old_y = old_index / old_meta.width_cells;

    const double world_x =
      old_meta.origin_x_m +
      (static_cast<double>(old_x) + 0.5) * old_meta.resolution_m;

    const double world_y =
      old_meta.origin_y_m +
      (static_cast<double>(old_y) + 0.5) * old_meta.resolution_m;

    const auto maybe_new_index = worldToIndex(meta_, world_x, world_y);
    if (!maybe_new_index.has_value()) {
      continue;
    }

    const std::size_t new_index = maybe_new_index.value();

    if (std::abs(old_evidence_scores[old_index]) >
      std::abs(new_evidence_scores[new_index]))
    {
      new_evidence_scores[new_index] = old_evidence_scores[old_index];
      new_state_transition_counts[new_index] = old_state_transition_counts[old_index];
      new_observed[new_index] = old_observed[old_index];
      new_cell_states[new_index] = old_cell_states[old_index];
    } else if (new_observed[new_index] == 0U) {
      new_evidence_scores[new_index] = old_evidence_scores[old_index];
      new_state_transition_counts[new_index] = old_state_transition_counts[old_index];
      new_observed[new_index] = old_observed[old_index];
      new_cell_states[new_index] = old_cell_states[old_index];
    }
  }

  obstacle_hits_per_update_.assign(new_cell_count, 0);
  ground_hits_per_update_.assign(new_cell_count, 0);

  evidence_scores_ = std::move(new_evidence_scores);
  state_transition_counts_ = std::move(new_state_transition_counts);
  observed_ = std::move(new_observed);
  cell_states_ = std::move(new_cell_states);

  clearance_m_.assign(new_cell_count, 0.0F);

  heading_feasible_.assign(std::max<std::size_t>(1, config_.heading_bin_count), {});
  for (auto & heading_mask : heading_feasible_) {
    heading_mask.assign(new_cell_count, 0U);
  }

  buildFootprintStencils();
  classifyCells();
  computeClearanceMeters();
  computeHeadingFeasibleMasks();
}

void Builder::reset()
{
  std::fill(obstacle_hits_per_update_.begin(), obstacle_hits_per_update_.end(), 0);
  std::fill(ground_hits_per_update_.begin(), ground_hits_per_update_.end(), 0);
  std::fill(evidence_scores_.begin(), evidence_scores_.end(), 0);
  std::fill(state_transition_counts_.begin(), state_transition_counts_.end(), 0U);
  std::fill(observed_.begin(), observed_.end(), 0U);
  std::fill(cell_states_.begin(), cell_states_.end(), CellState::Unknown);
  std::fill(clearance_m_.begin(), clearance_m_.end(), 0.0F);

  for (auto & heading_mask : heading_feasible_) {
    std::fill(heading_mask.begin(), heading_mask.end(), 0U);
  }
}

void Builder::resizeBuffers()
{
  obstacle_hits_per_update_.assign(approach_map::cellCount(meta_), 0);
  ground_hits_per_update_.assign(approach_map::cellCount(meta_), 0);
  evidence_scores_.assign(approach_map::cellCount(meta_), 0);
  state_transition_counts_.assign(approach_map::cellCount(meta_), 0U);
  observed_.assign(approach_map::cellCount(meta_), 0U);
  cell_states_.assign(approach_map::cellCount(meta_), CellState::Unknown);
  clearance_m_.assign(approach_map::cellCount(meta_), 0.0F);

  heading_feasible_.assign(std::max<std::size_t>(1, config_.heading_bin_count), {});
  for (auto & heading_mask : heading_feasible_) {
    heading_mask.assign(approach_map::cellCount(meta_), 0U);
  }
}

void Builder::beginUpdate()
{
  std::fill(obstacle_hits_per_update_.begin(), obstacle_hits_per_update_.end(), 0);
  std::fill(ground_hits_per_update_.begin(), ground_hits_per_update_.end(), 0);
}

void Builder::addGroundObservation(double x_m, double y_m)
{
  if (!std::isfinite(x_m) || !std::isfinite(y_m)) {
    return;
  }

  const auto maybe_index = worldToIndex(meta_, x_m, y_m);
  if (!maybe_index.has_value()) {
    return;
  }

  ++ground_hits_per_update_[maybe_index.value()];
}

void Builder::addObstacleObservation(double x_m, double y_m)
{
  if (!std::isfinite(x_m) || !std::isfinite(y_m)) {
    return;
  }

  const auto maybe_index = worldToIndex(meta_, x_m, y_m);
  if (!maybe_index.has_value()) {
    return;
  }

  ++obstacle_hits_per_update_[maybe_index.value()];
}

void Builder::addGroundObservations(const std::vector<XYPoint> & points)
{
  for (const auto & point : points) {
    addGroundObservation(point.x_m, point.y_m);
  }
}

void Builder::addObstacleObservations(const std::vector<XYPoint> & points)
{
  for (const auto & point : points) {
    addObstacleObservation(point.x_m, point.y_m);
  }
}

void Builder::endUpdate()
{
  integrateEvidence();
  classifyCells();
  computeClearanceMeters();
  computeHeadingFeasibleMasks();
}

const Config & Builder::config() const
{
  return config_;
}

const GridMeta & Builder::gridMeta() const
{
  return meta_;
}

Origin Builder::origin() const
{
  return Origin{meta_.origin_x_m, meta_.origin_y_m};
}

std::size_t Builder::cellCount() const
{
  return approach_map::cellCount(meta_);
}

double Builder::headingAngleRad(std::size_t heading_bin) const
{
  const std::size_t bin_count = std::max<std::size_t>(1, heading_feasible_.size());
  return kTwoPi * static_cast<double>(heading_bin % bin_count) / static_cast<double>(bin_count);
}

const std::vector<int> & Builder::obstacleHitsPerUpdate() const
{
  return obstacle_hits_per_update_;
}

const std::vector<int> & Builder::groundHitsPerUpdate() const
{
  return ground_hits_per_update_;
}

const std::vector<int> & Builder::evidenceScores() const
{
  return evidence_scores_;
}

const std::vector<uint32_t> & Builder::stateTransitionCounts() const
{
  return state_transition_counts_;
}

const std::vector<uint8_t> & Builder::observedMask() const
{
  return observed_;
}

const std::vector<CellState> & Builder::cellStates() const
{
  return cell_states_;
}

const std::vector<float> & Builder::clearanceMeters() const
{
  return clearance_m_;
}

const std::vector<uint8_t> & Builder::headingFeasibleMask(std::size_t heading_bin) const
{
  return heading_feasible_[heading_bin % heading_feasible_.size()];
}

GridDataI8 Builder::buildObstacleLayer() const
{
  std::vector<int8_t> values(cellCount(), -1);

  for (std::size_t i = 0; i < cellCount(); ++i) {
    if (cell_states_[i] == CellState::Obstacle) {
      values[i] = 100;
    } else if (cell_states_[i] == CellState::Free) {
      values[i] = 0;
    } else if (observed_[i] != 0U) {
      values[i] = 50;
    }
  }

  return makeGridDataI8(meta_, std::move(values));
}

GridDataF32 Builder::buildClearanceLayer() const
{
  std::vector<float> values(cellCount(), -1.0F);

  for (std::size_t i = 0; i < cellCount(); ++i) {
    if (observed_[i] == 0U) {
      continue;
    }

    values[i] = clearance_m_[i];
  }

  return makeGridDataF32(meta_, std::move(values));
}

GridDataI8 Builder::buildHeadingFeasibleLayer(std::size_t heading_bin) const
{
  std::vector<int8_t> values(cellCount(), -1);
  const auto & feasible_mask = headingFeasibleMask(heading_bin);

  for (std::size_t i = 0; i < cellCount(); ++i) {
    if (observed_[i] == 0U) {
      continue;
    }

    values[i] = feasible_mask[i] != 0U ? 100 : 0;
  }

  return makeGridDataI8(meta_, std::move(values));
}

}  // namespace approach_map
