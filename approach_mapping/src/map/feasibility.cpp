/**
 * @file feasibility.cpp
 * @brief
 * 로봇이 특정 위치에서 설 수 있는 자세를 판별하기 위한 코드.
 */

#include "approach_map/map/builder.hpp"

#include <algorithm>
#include <cmath>

namespace approach_map
{

namespace
{

constexpr double kSqrt2 = 1.4142135623730951;

}  // namespace

/**
 * @brief
 * 회전된 로봇 footprint 모양을 미리 만들어두는 함수
 * @details
 * heading bin개수만큼 stencil을 만들어둔다.
 * 각 stencil은 footprint의 모양을 보수적으로 확장한 형태로, 로봇이 해당 위치에 있을 때, 장애물 셀과의 간섭이 없는지를 확인하기 위한 오프셋들의 집합이다.
 * conservative margin은 footprint의 실제 크기에 추가적으로 더해지는 여유 공간으로, 센서 노이즈나 위치 추정 오차로 인해 로봇이 실제로 footprint보다 더 넓은 공간을 차지할 수 있음을 고려한다.
 * max extent는 footprint의 대각선 길이로, footprint가 회전되어 있을 때 가장 멀리 떨어진 점까지의 거리를 나타낸다. 
 * max offset cells는 max extent를 grid의 해상도로 나눈 값으로, footprint가 회전되어 있을 때 필요한 최대 오프셋 셀 수를 나타낸다.
 * 각 heading bin에 대해서, 회전된 footprint의 모양을 따라 오프셋 셀들을 계산하여 stencil에 저장한다.
 * 이렇게 만들어진 stencil은 나중에 로봇이 특정 위치에 있을 때, 해당 위치에서의 footprint가 장애물 셀과 간섭이 없는지를 빠르게 확인하는 데 사용된다.
 */
void Builder::buildFootprintStencils()
{
  footprint_stencils_.assign(std::max<std::size_t>(1, config_.heading_bin_count), {});

  const double conservative_margin_m =
    config_.footprint_margin_m + 0.5 * meta_.resolution_m * kSqrt2;
  const double half_length_m = 0.5 * config_.footprint_length_m + conservative_margin_m;
  const double half_width_m = 0.5 * config_.footprint_width_m + conservative_margin_m;
  const double max_extent_m = std::hypot(half_length_m, half_width_m);
  const int max_offset_cells = static_cast<int>(std::ceil(max_extent_m / meta_.resolution_m));

  for (std::size_t heading_bin = 0; heading_bin < footprint_stencils_.size(); ++heading_bin) {
    const double theta_rad = headingAngleRad(heading_bin);
    const double cos_theta = std::cos(theta_rad);
    const double sin_theta = std::sin(theta_rad);
    auto & stencil = footprint_stencils_[heading_bin];

    for (int dy = -max_offset_cells; dy <= max_offset_cells; ++dy) {
      for (int dx = -max_offset_cells; dx <= max_offset_cells; ++dx) {
        const double world_x_m = static_cast<double>(dx) * meta_.resolution_m;
        const double world_y_m = static_cast<double>(dy) * meta_.resolution_m;

        const double body_x_m = cos_theta * world_x_m + sin_theta * world_y_m;
        const double body_y_m = -sin_theta * world_x_m + cos_theta * world_y_m;

        if (std::abs(body_x_m) <= half_length_m && std::abs(body_y_m) <= half_width_m) {
          stencil.push_back({dx, dy});
        }
      }
    }
  }
}

/**
 * @brief
 * 특정 셀과 특정 방향에서 로봇 footprint가 충돌 없이 들어가는지 검사
 * @details
 * 
 */
bool Builder::footprintFitsAt(std::size_t center_index, std::size_t heading_bin) const
{
  if (footprint_stencils_.empty()) {
    return false;
  }

  const std::size_t x_cell = center_index % meta_.width_cells;
  const std::size_t y_cell = center_index / meta_.width_cells;
  const auto & stencil = footprint_stencils_[heading_bin % footprint_stencils_.size()];

  for (const auto & offset : stencil) {
    const int sample_x = static_cast<int>(x_cell) + offset.dx;
    const int sample_y = static_cast<int>(y_cell) + offset.dy;
    if (!isInsideGrid(meta_, sample_x, sample_y)) {
      return false;
    }

    const std::size_t sample_index =
      flattenIndex(meta_, static_cast<std::size_t>(sample_x), static_cast<std::size_t>(sample_y));

    if (cell_states_[sample_index] == CellState::Obstacle) {
      return false;
    }

    if (
      config_.unknown_is_blocked_for_feasibility &&
      cell_states_[sample_index] != CellState::Free)
    {
      return false;
    }
  }

  return true;
}

/**
 * @brief 
 * 모든 heading bin에 대해, 모든 셀을 검사
 * 위의 footprintFitsAt 함수를 맵 전체에 적용
 */
void Builder::computeHeadingFeasibleMasks()
{
  for (std::size_t heading_bin = 0; heading_bin < heading_feasible_.size(); ++heading_bin) {
    auto & feasible_mask = heading_feasible_[heading_bin];
    std::fill(feasible_mask.begin(), feasible_mask.end(), 0U);

    for (std::size_t i = 0; i < cellCount(); ++i) {
      if (cell_states_[i] != CellState::Free) {
        continue;
      }

      feasible_mask[i] = footprintFitsAt(i, heading_bin) ? 1U : 0U;
    }
  }
}

/**
 * @brief
 * 로봇이 실제로 지나온(visited) 셀을 Free로 강제하는 함수.
 * @details
 * 로봇이 물리적으로 점유/통과한 위치이므로 희소하거나 노이즈가 섞인 관측과 무관하게
 * 확실히 비어 있는 공간으로 간주한다. classifyCells 이후에 호출되어, 순간적인 노이즈로
 * 장애물로 분류된 visited 셀을 다시 Free로 되돌린다.
 */
void Builder::enforceVisitedFree()
{
  for (std::size_t i = 0; i < cellCount(); ++i) {
    if (visited_[i] != 0U) {
      cell_states_[i] = CellState::Free;
    }
  }
}

/**
 * @brief
 * visited 셀을 모든 heading bin에서 feasible로 강제하는 함수.
 * @details
 * 로봇이 지나온 경로는 항상 도달 가능한 seed와 연결성을 보장해야 하므로,
 * footprint 충돌 검사와 무관하게 feasible로 OR 처리한다.
 */
void Builder::applyVisitedFeasible()
{
  for (auto & feasible_mask : heading_feasible_) {
    for (std::size_t i = 0; i < cellCount(); ++i) {
      if (visited_[i] != 0U) {
        feasible_mask[i] = 1U;
      }
    }
  }
}

}  // namespace approach_map
