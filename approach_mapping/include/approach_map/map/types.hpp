#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace approach_map
{

/**
 * @brief 2D point in map coordinates.
 */
struct XYPoint
{
  double x_m{0.0};
  double y_m{0.0};
};

/**
 * @brief Grid origin expressed in meters.
 */
struct Origin
{
  double x_m{0.0};
  double y_m{0.0};
};

/**
 * @brief Semantic state assigned to each grid cell.
 */
enum class CellState : uint8_t
{
  Unknown = 0,
  Free = 1,
  Obstacle = 2,
};

/**
 * @brief Common metadata shared by all grid layers.
 */
struct GridMeta
{
  std::size_t width_cells{0};
  std::size_t height_cells{0};
  double resolution_m{0.10};
  double origin_x_m{-10.0};
  double origin_y_m{-10.0};
};

/**
 * @brief Integer-valued grid layer container.
 */
struct GridDataI8
{
  GridMeta meta;
  std::vector<int8_t> values;
};

/**
 * @brief Floating-point grid layer container.
 */
struct GridDataF32
{
  GridMeta meta;
  std::vector<float> values;
};

/**
 * @brief Configuration for evidence mapping and heading-aware feasibility.
 */
struct Config
{
  double resolution_m{0.10};
  double width_m{5.0};
  double height_m{5.0};

  int obstacle_hit_cap_per_update{3};
  int ground_hit_cap_per_update{3};
  int obstacle_weight{2};
  int ground_weight{1};
  int evidence_clip_value{100};
  int occupied_score_threshold{10};
  int free_score_threshold{8};

  double footprint_length_m{0.90};
  double footprint_width_m{0.60};
  double footprint_margin_m{0.05};
  std::size_t heading_bin_count{16};
  bool unknown_is_blocked_for_feasibility{true};
};

/**
 * @brief Build grid metadata from the provided mapping configuration.
 * @param config User-defined map configuration.
 * @param origin Grid origin in meters.
 * @return Grid metadata derived from map size, origin, and resolution.
 */
GridMeta makeGridMeta(const Config & config, const Origin & origin);

/**
 * @brief Compute the total number of cells in the grid.
 * @param meta Grid metadata describing the map dimensions.
 * @return Total flattened cell count.
 */
std::size_t cellCount(const GridMeta & meta);

/**
 * @brief Check whether a cell coordinate lies inside the grid bounds.
 * @param meta Grid metadata describing the map dimensions.
 * @param x_cell X coordinate in cell units.
 * @param y_cell Y coordinate in cell units.
 * @return True if the coordinate is valid for the grid.
 */
bool isInsideGrid(const GridMeta & meta, int x_cell, int y_cell);

/**
 * @brief Convert 2D cell coordinates into a row-major flattened index.
 * @param meta Grid metadata describing the map dimensions.
 * @param x_cell X coordinate in cell units.
 * @param y_cell Y coordinate in cell units.
 * @return Flattened row-major index.
 */
std::size_t flattenIndex(const GridMeta & meta, std::size_t x_cell, std::size_t y_cell);

/**
 * @brief Convert world coordinates in meters into a grid index.
 * @param meta Grid metadata describing the map geometry.
 * @param x_m X position in meters.
 * @param y_m Y position in meters.
 * @return Flattened grid index, or std::nullopt if the point is outside the grid.
 */
std::optional<std::size_t> worldToIndex(const GridMeta & meta, double x_m, double y_m);

/**
 * @brief Wrap an int8 cell buffer with grid metadata.
 * @param meta Grid metadata describing the map geometry.
 * @param values Flattened int8 values in row-major order.
 * @return Integer grid layer with attached metadata.
 */
GridDataI8 makeGridDataI8(const GridMeta & meta, std::vector<int8_t> values);

/**
 * @brief Wrap a float cell buffer with grid metadata.
 * @param meta Grid metadata describing the map geometry.
 * @param values Flattened float values in row-major order.
 * @return Floating-point grid layer with attached metadata.
 */
GridDataF32 makeGridDataF32(const GridMeta & meta, std::vector<float> values);

}  // namespace approach_map
