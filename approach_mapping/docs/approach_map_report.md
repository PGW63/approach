# Approach Interaction Map 설계 보고서

## 1. 목적
본 맵은 일반적인 Nav2 주행 맵과 별개로, 물체 근처까지 더 가까이 접근하기 위한 전용 맵을 만드는 것이 목적이다.

- 먼 거리 이동은 기존 Nav2가 담당한다.
- 물체 근처의 세밀한 접근은 본 맵이 담당한다.
- 입력 포인트는 다른 패키지에서 이미 `ground`와 `obstacle`로 분리되어 들어온다고 가정한다.
- 대회 환경은 평평한 바닥이라고 가정한다.

## 2. 핵심 아이디어
맵은 한 번 관측된 정보를 쉽게 지우지 않는 `persistent map` 구조로 유지한다.

- 관측이 안 된다고 해서 셀을 지우지 않는다.
- 반대 증거가 충분히 들어올 때만 셀 상태를 바꾼다.
- 셀 상태는 `unknown`, `free`, `obstacle` 세 가지로 관리한다.

이 방식은 로봇이 다른 구역으로 이동한 뒤에도 이전에 본 공간이 사라지지 않는다는 장점이 있다.

## 3. 입력과 출력
### 입력
- `ground 2D points`
- `obstacle 2D points`
- 맵 파라미터
  - resolution
  - width, height
  - origin
  - obstacle/ground 가중치
  - state threshold
  - 로봇 footprint 크기
  - heading bin 수

### 출력
- `Obstacle Map`
- `Clearance Map`
- `Heading-Feasible Map`

## 4. 셀 상태 관리 방식
각 셀은 다음 정보를 가진다.

1. `obstacle_hits_per_update`
   현재 업데이트에서 obstacle로 들어온 포인트 수
2. `ground_hits_per_update`
   현재 업데이트에서 ground로 들어온 포인트 수
3. `evidence_score`
   obstacle/free 증거를 누적한 signed score
4. `observed`
   한 번이라도 관측된 셀인지 여부
5. `cell_state`
   `unknown / free / obstacle`

## 5. 업데이트 알고리즘
업데이트는 다음 순서로 진행된다.

1. `beginUpdate()`
   현재 프레임용 hit count를 0으로 초기화한다.
2. `addGroundObservation()`, `addObstacleObservation()`
   각 포인트를 해당 셀의 hit count에 누적한다.
3. `endUpdate()`
   hit count를 evidence score에 반영하고, state를 갱신한다.

점수 갱신식은 아래와 같다.

```text
score[c] =
  clip(
    score[c]
    + obstacle_weight * min(obstacle_hits[c], obstacle_hit_cap)
    - ground_weight   * min(ground_hits[c],   ground_hit_cap),
    -evidence_clip_value,
    +evidence_clip_value
  )
```

셀 상태는 hysteresis 방식으로 정한다.

```text
if score[c] >= occupied_score_threshold:
    state[c] = obstacle
else if score[c] <= -free_score_threshold:
    state[c] = free
else:
    state[c] = previous state
```

이 방식의 의미는 다음과 같다.

- 약한 관측만 들어왔을 때는 상태가 쉽게 흔들리지 않는다.
- obstacle이었던 셀은 ground 증거가 충분히 들어올 때만 free로 바뀐다.
- free였던 셀도 obstacle 증거가 충분할 때만 obstacle로 바뀐다.

## 6. Obstacle Map
Obstacle Map은 셀 상태를 그대로 2D 맵으로 만든다.

- `obstacle` 셀: 100
- `free` 셀: 0
- 관측은 됐지만 아직 확정되지 않은 셀: 50
- 한 번도 관측되지 않은 셀: -1

이 맵은 Nav2의 costmap과 직접 동일한 의미는 아니지만, interaction 전용 planning의 기본 입력으로 쓸 수 있다.

## 7. Clearance Map
Clearance Map은 obstacle 셀로부터의 2D 거리값을 저장한 맵이다.

- obstacle 셀은 0
- free 셀은 가까운 obstacle까지의 거리
- unknown 셀은 필요에 따라 무시하거나 blocked로 취급 가능

이 맵은 “장애물과 얼마나 떨어져 있는가”를 바로 볼 수 있게 해준다.

## 8. Feasible Map
로봇이 원형이 아니므로, clearance만으로 주행 가능 여부를 판단하면 부정확하다.
따라서 `heading`마다 회전된 직사각형 footprint를 사용해 feasible 여부를 계산한다.

계산 방식은 다음과 같다.

1. heading bin마다 회전된 footprint stencil을 미리 만든다.
2. 각 셀을 중심으로 footprint를 슬라이딩한다.
3. footprint 아래 셀이 모두 `free`이면 그 heading에서 feasible로 판단한다.

즉, 최종 feasible 값은 `cell` 하나가 아니라 `(x, y, theta)` pose 기준으로 결정된다.

## 9. 제자리 회전이 안 되는 로봇에 대한 해석
현재 feasible map은 “그 자세로 그 위치에 설 수 있는가”를 검사한다.
하지만 “현재 로봇 자세에서 그 위치까지 실제로 갈 수 있는가”는 아직 포함하지 않는다.

따라서 실제 상호작용 직전에는 한 단계가 더 필요하다.

- 후보 interaction pose 생성
- 각 후보에 대해 최소 회전 반경을 반영한 경로 가능성 검사
- 예: Dubins, Reeds-Shepp 기반 reachable check

즉 본 맵의 역할은 `collision-feasible pose`를 빠르게 만드는 것이고,
최종 접근 가능성은 별도의 local planner 또는 interaction planner가 판단하는 구조가 적절하다.

## 10. 최종 권장 구조
권장 구조는 다음과 같다.

1. 전처리 패키지
   `3D LiDAR -> ground / obstacle 분리`
2. 본 맵 패키지
   `persistent evidence map -> obstacle map / clearance map / heading-feasible map 생성`
3. interaction planner
   `target 주변 후보 pose 생성 -> reachable check -> 최종 접근 pose 선택`

## 11. 코드 구조
코드는 아래처럼 역할별로 분리한다.

- `include/approach_map/map/types.hpp`
  파라미터, 기본 타입, grid helper 선언
- `src/map/types.cpp`
  grid meta, index 계산, grid data 생성 구현
- `include/approach_map/map/builder.hpp`
  공개 맵 빌더 API
- `src/map/builder.cpp`
  빌더 초기화, 업데이트 흐름, 레이어 생성
- `src/map/evidence.cpp`
  evidence score 누적과 state 판정
- `src/map/clearance.cpp`
  obstacle 기준 거리 계산
- `src/map/feasibility.cpp`
  footprint stencil과 heading별 feasible 계산

## 12. 결론
최종적으로 본 맵은 다음 성격을 가진다.

- 한 번 본 공간을 쉽게 잃지 않는 persistent map
- obstacle/free를 signed evidence로 안정적으로 판정
- 직사각형 footprint와 heading을 반영한 feasible 계산
- Nav2와 별개로 상호작용 근접 접근을 위한 기반 맵

이 구조는 flat한 대회 환경에서 구현 난이도와 안정성의 균형이 가장 좋다.
