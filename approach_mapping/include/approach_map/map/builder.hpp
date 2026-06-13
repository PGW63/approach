#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "approach_map/map/types.hpp"

namespace approach_map
{

class Builder
{
public:
  /**
   * @brief Construct a map builder with the given configuration.
   * @param config Mapping and footprint configuration.
   * @param origin Grid origin in meters.
   */
  Builder(const Config & config, const Origin & origin);

  /**
   * @brief Reconfigure the builder and rebuild internal buffers.
   * @param config Updated mapping and footprint configuration.
   * @param origin Grid origin in meters.
   */
  void reconfigure(const Config & config, const Origin & origin);

  /**
   * @brief Change only the grid origin and reset internal buffers.
   * @param origin Grid origin in meters.
   */
  void setOrigin(const Origin & origin);

  /**
   * @brief Change the grid origin while preserving accumulated map evidence.
   *
   * Existing persistent map evidence is reprojected from the old grid to the new
   * grid using each old cell center's world coordinate. Cells outside the new grid
   * are discarded. Derived layers such as clearance and heading feasibility are
   * recomputed after the shift.
   *
   * @param origin New grid origin in meters.
   */
  void shiftOriginPreserveEvidence(const Origin & origin);

  /**
   * @brief Reset all accumulated evidence and derived map layers.
   */
  void reset();

  /**
   * @brief Start a new update cycle by clearing per-update hit buffers.
   */
  void beginUpdate();

  /**
   * @brief Add one ground observation to the current update.
   * @param x_m Ground point x coordinate in meters.
   * @param y_m Ground point y coordinate in meters.
   */
  void addGroundObservation(double x_m, double y_m);

  /**
   * @brief Add one obstacle observation to the current update.
   * @param x_m Obstacle point x coordinate in meters.
   * @param y_m Obstacle point y coordinate in meters.
   */
  void addObstacleObservation(double x_m, double y_m);

  /**
   * @brief Add multiple ground observations to the current update.
   * @param points Ground points expressed in map coordinates.
   */
  void addGroundObservations(const std::vector<XYPoint> & points);

  /**
   * @brief Add multiple obstacle observations to the current update.
   * @param points Obstacle points expressed in map coordinates.
   */
  void addObstacleObservations(const std::vector<XYPoint> & points);

  /**
   * @brief Mark a disc of cells around the robot as visited (known-free).
   *
   * The robot physically occupies/traverses this area, so the cells are treated
   * as definitely free and feasible regardless of sparse or noisy observations.
   * The visited mask is persistent and accumulates the full robot trajectory,
   * guaranteeing a reachable seed and connectivity for downstream planning.
   *
   * @param x_m Robot center x coordinate in map meters.
   * @param y_m Robot center y coordinate in map meters.
   * @param radius_m Disc radius in meters around the robot center.
   */
  void markVisited(double x_m, double y_m, double radius_m);

  /**
   * @brief Finalize the current update and rebuild derived layers.
   */
  void endUpdate();

  /**
   * @brief Get the active builder configuration.
   * @return Immutable reference to the current configuration.
   */
  const Config & config() const;

  /**
   * @brief Get the current grid metadata.
   * @return Immutable reference to the derived grid geometry.
   */
  const GridMeta & gridMeta() const;

  /**
   * @brief Get the current grid origin.
   * @return Current origin in meters.
   */
  Origin origin() const;

  /**
   * @brief Get the total number of cells in the current grid.
   * @return Flattened row-major cell count.
   */
  std::size_t cellCount() const;

  /**
   * @brief Convert a heading bin to an angle in radians.
   * @param heading_bin Heading bin index.
   * @return Heading angle corresponding to the given bin.
   */
  double headingAngleRad(std::size_t heading_bin) const;

  /**
   * @brief Access obstacle hits collected during the latest update.
   * @return Per-cell obstacle hit counts.
   */
  const std::vector<int> & obstacleHitsPerUpdate() const;

  /**
   * @brief Access ground hits collected during the latest update.
   * @return Per-cell ground hit counts.
   */
  const std::vector<int> & groundHitsPerUpdate() const;

  /**
   * @brief Access persistent signed evidence values for each cell.
   * @return Per-cell evidence scores.
   */
  const std::vector<int> & evidenceScores() const;

  /**
   * @brief Access the number of free/obstacle flips observed for each cell.
   * @return Per-cell transition counts between stable semantic states.
   */
  const std::vector<uint32_t> & stateTransitionCounts() const;

  /**
   * @brief Access the mask of cells observed at least once.
   * @return Per-cell observed flags.
   */
  const std::vector<uint8_t> & observedMask() const;

  /**
   * @brief Access the mask of cells the robot has visited (known-free).
   * @return Per-cell visited flags.
   */
  const std::vector<uint8_t> & visitedMask() const;

  /**
   * @brief Access the current semantic state of each cell.
   * @return Per-cell classified state values.
   */
  const std::vector<CellState> & cellStates() const;

  /**
   * @brief Access obstacle clearance values in meters.
   * @return Per-cell clearance values.
   */
  const std::vector<float> & clearanceMeters() const;

  /**
   * @brief Access the feasible mask for a specific heading bin.
   * @param heading_bin Heading bin index.
   * @return Per-cell feasibility mask for the requested heading.
   */
  const std::vector<uint8_t> & headingFeasibleMask(std::size_t heading_bin) const;

  /**
   * @brief Build an integer obstacle layer from the current map state.
   * @return Encoded obstacle layer with metadata.
   */
  GridDataI8 buildObstacleLayer() const;

  /**
   * @brief Build a floating-point clearance layer from the current map state.
   * @return Clearance layer with metadata.
   */
  GridDataF32 buildClearanceLayer() const;

  /**
   * @brief Build a feasible layer for a specific heading bin.
   * @param heading_bin Heading bin index.
   * @return Encoded feasible layer with metadata.
   */
  GridDataI8 buildHeadingFeasibleLayer(std::size_t heading_bin) const;

private:
  struct OffsetCell
  {
    int dx{0};
    int dy{0};
  };

  void resizeBuffers();
  void integrateEvidence();
  void classifyCells();
  void computeClearanceMeters();
  void buildFootprintStencils();
  void computeHeadingFeasibleMasks();
  void enforceVisitedFree();
  void applyVisitedFeasible();
  bool footprintFitsAt(std::size_t center_index, std::size_t heading_bin) const;

  Config config_;
  GridMeta meta_;

  std::vector<int> obstacle_hits_per_update_;
  std::vector<int> ground_hits_per_update_;
  std::vector<int> evidence_scores_;
  std::vector<uint32_t> state_transition_counts_;
  std::vector<uint8_t> observed_;
  std::vector<uint8_t> visited_;
  std::vector<CellState> cell_states_;
  std::vector<float> clearance_m_;
  std::vector<std::vector<OffsetCell>> footprint_stencils_;
  std::vector<std::vector<uint8_t>> heading_feasible_;
};

}  // namespace approach_map
