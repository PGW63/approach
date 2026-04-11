#include "approach_map/map/builder.hpp"

#include <array>
#include <limits>
#include <queue>
#include <utility>

namespace approach_map
{

namespace
{

constexpr double kSqrt2 = 1.4142135623730951;

struct DistanceNode
{
  float distance_m;
  std::size_t index;
};

struct DistanceCompare
{
  bool operator()(const DistanceNode & lhs, const DistanceNode & rhs) const
  {
    return lhs.distance_m > rhs.distance_m;
  }
};

}  // namespace

/**
 * @brief
 * obstacle 셀로부터 각 셀까지의 근사 clerance를 계산한다.
 * 처음에는 모든 셀의 거리를 모르기에 inf로 초기화한다.
 * 가장 작은 거리부터 꺼내기 위해 우선순위 큐를 사용한다.
 * 장애물 셀은 0으로 초기화한다.
 * 주변 8방향을 정의하여 큐가 빌 때까지 거리를 퍼뜨린다.
 * 프라이어티 큐이므로, 항상 가장 작은 거리값을 가진 셀이
 * 큐로부터 나온다. 만약 그 셀의 거리값이 이미 계산된 값보다 크면, 무시한다.
 * 그렇지 않으면, 그 셀의 이웃 셀들에 대해서, 현재 셀의 거리값 + 이웃 셀과의 간격이 이웃 셀의 거리값보다 작은지 확인한다.
 * 작으면, 이웃 셀의 거리값을 업데이트하고, 큐에 넣는다.
 * 이렇게 하면, 장애물 셀로부터 점점 멀어지는 셀들의 거리가 정확하게 계산된다.
 * 이 알고리즘은 Dijkstra의 최단 경로 알고리즘과 유사하며, grid-based map에서 clearance를 계산하는 데 널리 사용된다.
 */
void Builder::computeClearanceMeters()
{
  const float inf = std::numeric_limits<float>::infinity();
  std::fill(clearance_m_.begin(), clearance_m_.end(), inf);

  std::priority_queue<DistanceNode, std::vector<DistanceNode>, DistanceCompare> queue;

  for (std::size_t i = 0; i < cellCount(); ++i) {
    if (cell_states_[i] == CellState::Obstacle) {
      clearance_m_[i] = 0.0F;
      queue.push({0.0F, i});
    }
  }

  const std::array<std::pair<int, int>, 8> deltas{
    std::make_pair(-1, 0), std::make_pair(1, 0), std::make_pair(0, -1), std::make_pair(0, 1),
    std::make_pair(-1, -1), std::make_pair(-1, 1), std::make_pair(1, -1), std::make_pair(1, 1)};

  while (!queue.empty()) {
    const DistanceNode current = queue.top();
    queue.pop();

    if (current.distance_m > clearance_m_[current.index]) {
      continue;
    }

    const std::size_t x_cell = current.index % meta_.width_cells;
    const std::size_t y_cell = current.index / meta_.width_cells;

    for (const auto & delta : deltas) {
      const int nx = static_cast<int>(x_cell) + delta.first;
      const int ny = static_cast<int>(y_cell) + delta.second;
      if (!isInsideGrid(meta_, nx, ny)) {
        continue;
      }

      const std::size_t neighbor_index =
        flattenIndex(meta_, static_cast<std::size_t>(nx), static_cast<std::size_t>(ny));
      const bool diagonal = delta.first != 0 && delta.second != 0;
      const float edge_cost_m =
        static_cast<float>(meta_.resolution_m * (diagonal ? kSqrt2 : 1.0));
      const float candidate_distance_m = current.distance_m + edge_cost_m;

      if (candidate_distance_m >= clearance_m_[neighbor_index]) {
        continue;
      }

      clearance_m_[neighbor_index] = candidate_distance_m;
      queue.push({candidate_distance_m, neighbor_index});
    }
  }
}

}  // namespace approach_map
