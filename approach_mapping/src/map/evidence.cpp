#include "approach_map/map/builder.hpp"

#include <algorithm>

namespace approach_map
{

  /**
   * @brief 
   * 각 셀을 돌면서 저장된 장애물 히트와 지면 히트를 기반으로 각셀이 
   * 장애물이 점유했는지 아니면 지면 공간인지를 확인하기 위해 점수를 측정하는
   * 함수이다.
   * std::min을 사용하는 이유는 업데이트 시 많은 포인트가 찍혔다고 가중치를
   * 많이 주는 것은 위험할 수 있다고 생각하기 떄문이다.
   * (ICP가 잘못 틀어지면 지면 공간인데 장애물 히트가 많이 찍힐 수 있다.)
   *
   */
void Builder::integrateEvidence()
{
  for (std::size_t i = 0; i < cellCount(); ++i) {
    const int obstacle_hits =
      std::min(obstacle_hits_per_update_[i], config_.obstacle_hit_cap_per_update);
    const int ground_hits = std::min(ground_hits_per_update_[i], config_.ground_hit_cap_per_update);

    if (obstacle_hits > 0 || ground_hits > 0) {
      observed_[i] = 1U;
    }

    const int delta_score = config_.obstacle_weight * obstacle_hits -
      config_.ground_weight * ground_hits;

    evidence_scores_[i] = std::clamp(
      evidence_scores_[i] + delta_score,
      -config_.evidence_clip_value,
      config_.evidence_clip_value);
  }
}

/** 
  * @brief
  * 위의 점수들에 기반하여 
  * 각 셀의 상태를 Obstacle 혹은 Free로 분류하는 함수이다.
  * 애매하면 상태유지를 진행한다.
  */
void Builder::classifyCells()
{
  for (std::size_t i = 0; i < cellCount(); ++i) {
    const CellState previous_state = cell_states_[i];

    if (evidence_scores_[i] >= config_.occupied_score_threshold) {
      cell_states_[i] = CellState::Obstacle;
    } else if (evidence_scores_[i] <= -config_.free_score_threshold) {
      cell_states_[i] = CellState::Free;
    }

    const bool stable_flip =
      (previous_state == CellState::Obstacle && cell_states_[i] == CellState::Free) ||
      (previous_state == CellState::Free && cell_states_[i] == CellState::Obstacle);

    if (stable_flip) {
      ++state_transition_counts_[i];
    }
  }
}

}  // namespace approach_map
