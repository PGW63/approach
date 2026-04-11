#include "approach_map/map/types.hpp"

#include <cmath>
#include <utility>

namespace approach_map
{

GridMeta makeGridMeta(const Config & config, const Origin & origin)
{
  GridMeta meta;
  meta.width_cells = static_cast<std::size_t>(std::ceil(config.width_m / config.resolution_m));
  meta.height_cells = static_cast<std::size_t>(std::ceil(config.height_m / config.resolution_m));
  meta.resolution_m = config.resolution_m;
  meta.origin_x_m = origin.x_m;
  meta.origin_y_m = origin.y_m;
  return meta;
}

std::size_t cellCount(const GridMeta & meta)
{
  return meta.width_cells * meta.height_cells;
}

bool isInsideGrid(const GridMeta & meta, int x_cell, int y_cell)
{
  return x_cell >= 0 && y_cell >= 0 && x_cell < static_cast<int>(meta.width_cells) &&
         y_cell < static_cast<int>(meta.height_cells);
}

std::size_t flattenIndex(const GridMeta & meta, std::size_t x_cell, std::size_t y_cell)
{
  return y_cell * meta.width_cells + x_cell;
}

std::optional<std::size_t> worldToIndex(const GridMeta & meta, double x_m, double y_m)
{
  const int x_cell = static_cast<int>(std::floor((x_m - meta.origin_x_m) / meta.resolution_m));
  const int y_cell = static_cast<int>(std::floor((y_m - meta.origin_y_m) / meta.resolution_m));

  if (!isInsideGrid(meta, x_cell, y_cell)) {
    return std::nullopt;
  }

  return flattenIndex(meta, static_cast<std::size_t>(x_cell), static_cast<std::size_t>(y_cell));
}

GridDataI8 makeGridDataI8(const GridMeta & meta, std::vector<int8_t> values)
{
  GridDataI8 grid;
  grid.meta = meta;
  grid.values = std::move(values);
  return grid;
}

GridDataF32 makeGridDataF32(const GridMeta & meta, std::vector<float> values)
{
  GridDataF32 grid;
  grid.meta = meta;
  grid.values = std::move(values);
  return grid;
}

}  // namespace approach_map
