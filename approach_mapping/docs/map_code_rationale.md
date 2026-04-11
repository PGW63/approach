# Map Code Rationale

## 목적
이 문서는 현재 `approach_map` 라이브러리의 각 함수가 왜 존재하는지, 왜 그런 방식으로 구현했는지, 어디가 아직 아쉬운지를 코드 관점에서 설명한다.

코드 기준 파일은 다음과 같다.

- `include/approach_map/map/types.hpp`
- `include/approach_map/map/builder.hpp`
- `src/map/types.cpp`
- `src/map/builder.cpp`
- `src/map/evidence.cpp`
- `src/map/clearance.cpp`
- `src/map/feasibility.cpp`

## 전체 구조
현재 구조는 세 층으로 나뉜다.

1. `types`
   맵 크기, 원점, 좌표 변환, grid container 같은 공통 타입
2. `builder`
   외부에서 호출하는 공개 API와 전체 update 흐름
3. `algorithm modules`
   evidence, clearance, feasibility 계산 로직

이렇게 나눈 이유는 다음과 같다.

- 외부 사용자는 `Builder`만 보면 된다.
- 맵 자료구조와 알고리즘을 분리해두면 수정 범위가 줄어든다.
- 이후 ROS node를 추가해도 코어 알고리즘은 그대로 재사용할 수 있다.

---

## 1. `src/map/types.cpp`

### `makeGridMeta(const Config & config, const Origin & origin)`
역할:
- 설정값과 origin을 받아 실제 grid shape를 계산한다.

왜 이렇게 했는가:
- `Config`에는 "맵을 어떻게 만들지"만 두고, 실제 index 계산에 필요한 `width_cells`, `height_cells`, `origin`은 `GridMeta`로 분리하기 위해서다.
- 이렇게 하면 `Config`는 알고리즘 파라미터, `GridMeta`는 실행 시점의 실제 맵 메타데이터 역할을 한다.

아쉬운 점:
- 현재는 `ceil(width / resolution)` 방식이다.
- width, height가 resolution의 배수가 아닐 때 마지막 셀이 약간 더 넓은 의미처럼 보일 수 있다.
- 지금은 실용상 문제 없지만, 향후에는 "실제 커버 영역" 문서화가 더 필요하다.

### `cellCount(const GridMeta & meta)`
역할:
- 2D 셀 수를 1D 버퍼 길이로 변환한다.

왜 이렇게 했는가:
- 모든 내부 버퍼를 flatten된 1D vector로 관리하기 때문이다.
- 계산이 단순하고 contiguous memory를 쓰므로 빠르다.

아쉬운 점:
- overflow 방어는 없다.
- 현재 맵 크기에서는 현실적으로 문제될 가능성이 낮다.

### `isInsideGrid(...)`
역할:
- 셀 좌표가 유효 범위 안인지 확인한다.

왜 이렇게 했는가:
- worldToIndex, clearance propagation, footprint collision check에서 반복적으로 쓰이기 때문이다.

아쉬운 점:
- 특별한 문제는 없다.
- 매우 자주 호출되므로 나중에 성능 병목이 생기면 inline 정도는 고려할 수 있다.

### `flattenIndex(...)`
역할:
- `(x, y)`를 row-major 1D index로 바꾼다.

왜 이렇게 했는가:
- 내부 저장 구조가 `std::vector` 하나이기 때문이다.

### `worldToIndex(...)`
역할:
- 미터 단위 좌표를 셀 index로 바꾼다.

왜 이렇게 했는가:
- 외부 입력은 모두 world 좌표 기반이기 때문이다.
- `origin + resolution` 기반의 가장 기본적인 occupancy/grid 방식이다.

아쉬운 점:
- 경계 셀에서 `floor()` 기준이므로 exactly-on-boundary 케이스 정의가 중요하다.
- 지금은 일반적인 grid convention으로 충분하다.

### `makeGridDataI8`, `makeGridDataF32`
역할:
- 메타데이터와 raw buffer를 한 묶음으로 반환한다.

왜 이렇게 했는가:
- 외부에서 layer를 받을 때 `vector`와 메타정보를 따로 관리하지 않도록 하기 위해서다.

---

## 2. `src/map/builder.cpp`

### `Builder::Builder(...)`
역할:
- 생성 즉시 usable state로 만든다.

왜 이렇게 했는가:
- 생성 후 별도 init 호출을 강제하지 않기 위해서다.
- `reconfigure()`를 재사용해서 초기화 경로를 하나로 통일했다.

### `reconfigure(...)`
역할:
- 새 설정과 새 origin으로 내부 상태를 모두 다시 만든다.

왜 이렇게 했는가:
- `resolution`, `size`, `origin`, `heading_bin_count`가 바뀌면 buffer shape와 stencil이 모두 바뀌기 때문이다.

중요:
- 이 함수는 사실상 "맵 전체 재생성"이다.

아쉬운 점:
- 기존 evidence를 보존하는 reconfigure는 없다.
- 지금은 단순성이 더 중요하다고 판단했다.

### `setOrigin(...)`
역할:
- origin만 바꾸고 내부 버퍼를 다시 만든다.

왜 이렇게 했는가:
- origin은 index 체계를 직접 바꾸므로, 단순 메타데이터 수정으로 끝나면 안 된다.
- 잘못하면 기존 evidence가 다른 위치에 잘못 대응된다.

중요:
- 현재 구현에서 origin 변경은 reset 성격이다.

아쉬운 점:
- "맵을 평행이동하며 기존 evidence를 옮기는 기능"은 아직 없다.
- moving local map이 필요하면 나중에 따로 구현해야 한다.

### `reset()`
역할:
- evidence, observed, state, clearance, feasible을 모두 초기화한다.

왜 이렇게 했는가:
- 실험 중 강제 초기화, 새 세션 시작, origin 변경 직후 재시작 같은 상황이 필요하기 때문이다.

### `resizeBuffers()`
역할:
- meta에 맞게 모든 내부 버퍼 크기를 맞추고 초기값을 넣는다.

왜 이렇게 했는가:
- size 관리가 여러 함수에 퍼지면 실수하기 쉽다.
- buffer shape 변경 로직을 한 군데에 몰아두기 위해서다.

아쉬운 점:
- `cellCount(meta_)`를 여러 번 반복 호출한다.
- 성능 문제는 거의 없지만 코드상 약간 장황하다.

### `beginUpdate()`
역할:
- 이번 frame에서만 쓰는 hit buffer를 0으로 초기화한다.

왜 이렇게 했는가:
- persistent evidence와 per-update raw hits를 분리하기 위해서다.
- 한 프레임 안의 관측과 누적 상태를 섞지 않으려는 의도다.

### `addGroundObservation(...)`, `addObstacleObservation(...)`
역할:
- 점 하나를 현재 update의 ground/obstacle hit로 누적한다.

왜 이렇게 했는가:
- 외부 전처리 노드가 point를 하나씩 넣든, vector로 넣든 대응하기 쉽다.
- NaN/inf와 out-of-grid 포인트를 이 단계에서 바로 버린다.

아쉬운 점:
- 현재는 hit count만 올린다.
- raycasting이나 free-space tracing은 아직 없다.

### `addGroundObservations(...)`, `addObstacleObservations(...)`
역할:
- 여러 점을 한 번에 넣는다.

왜 이렇게 했는가:
- 외부 인터페이스를 단순화하려는 목적이다.

### `endUpdate()`
역할:
- 한 프레임의 raw hits를 반영해 최종 layer를 갱신한다.

순서:
1. `integrateEvidence()`
2. `classifyCells()`
3. `computeClearanceMeters()`
4. `computeHeadingFeasibleMasks()`

왜 이렇게 했는가:
- evidence가 먼저 state가 되고
- state가 먼저 obstacle/free를 만들고
- obstacle/free가 있어야 clearance가 계산되고
- clearance/state가 있어야 footprint feasibility를 계산할 수 있기 때문이다.

### getter들과 `build*Layer()`
역할:
- 내부 raw state를 직접 보거나
- 외부에서 바로 사용 가능한 grid layer 형태로 내보낸다.

왜 이렇게 했는가:
- 디버그와 실행 인터페이스를 동시에 지원하기 위해서다.
- 예를 들어 디버그용으로는 `evidenceScores()`, 실제 사용용으로는 `buildObstacleLayer()`를 쓸 수 있다.

아쉬운 점:
- 아직 visualization 전용 helper는 없다.
- ROS 메시지 변환은 추후 adapter 계층에서 하는 것이 좋다.

---

## 3. `src/map/evidence.cpp`

### `integrateEvidence()`
역할:
- 한 프레임에서 받은 raw hit를 persistent evidence score로 누적한다.

왜 이렇게 했는가:
- decay를 쓰지 않고도 과거 관측을 유지하기 위해서다.
- obstacle와 ground를 별도 score 두 개로 누적하는 대신 signed score 하나로 합쳐서 상태 전환을 단순화했다.

핵심 이유:
- `obstacle_score`, `ground_score`를 둘 다 영구 누적하면 둘 다 커져서 나중에 flip이 잘 안 된다.
- signed score는 반대 증거가 들어오면 자연스럽게 되돌릴 수 있다.

`min(hit, cap)`을 쓰는 이유:
- 한 프레임에 point density가 높다고 과도한 weight를 먹지 않게 하기 위해서다.
- 센서 밀도보다 "관측되었다"는 사실이 더 중요하다고 본다.

아쉬운 점:
- hit quality는 아직 고려하지 않는다.
- 거리, incidence angle, point confidence 같은 것은 아직 없다.

### `classifyCells()`
역할:
- evidence score를 `Unknown / Free / Obstacle` 상태로 바꾼다.

왜 이렇게 했는가:
- 실행 단계에서는 score보다 state가 더 직관적이기 때문이다.
- threshold 기반 hysteresis를 쓰면 state flicker를 줄일 수 있다.

중요:
- threshold 사이 영역에서는 상태를 바꾸지 않는다.
- 즉 이전 상태를 유지한다.

아쉬운 점:
- 현재는 explicit한 `else keep previous state` 코드가 없고, 구현상 그대로 남는 형태다.
- 동작은 맞지만 문맥 없이 보면 의도가 바로 읽히지 않을 수 있다.

---

## 4. `src/map/clearance.cpp`

### `computeClearanceMeters()`
역할:
- obstacle 셀로부터 각 셀까지의 근사 clearance를 계산한다.

왜 이렇게 했는가:
- obstacle map만으로는 “얼마나 가까운지”를 판단하기 어렵다.
- interaction pose 후보나 안전 여유 판단에는 distance layer가 유용하다.

구현 방식:
- obstacle을 seed로 넣고
- 8-neighborhood로 priority queue 기반 전파를 한다.

왜 이렇게 했는가:
- 구현이 단순하고 robust하다.
- 정확한 Euclidean distance transform보다 조금 무겁지만 이해하기 쉽고 확장도 쉽다.

아쉬운 점:
- strict한 exact EDT는 아니다.
- 다만 현재 목적에서는 근사 clearance로 충분할 가능성이 높다.

추후 개선 후보:
- exact EDT
- unknown 셀을 별도 정책으로 처리
- target object와 static obstacle 분리

---

## 5. `src/map/feasibility.cpp`

### `buildFootprintStencils()`
역할:
- heading별 로봇 footprint mask를 미리 계산한다.

왜 이렇게 했는가:
- feasibility 계산 때마다 footprint를 다시 만들면 비효율적이다.
- heading bin 수가 정해져 있으므로 precompute가 자연스럽다.

`conservative_margin`을 넣은 이유:
- cell discretization 오차를 조금 보수적으로 흡수하기 위해서다.
- 경계에 걸친 obstacle을 놓치는 위험을 줄이려는 목적이다.

아쉬운 점:
- footprint가 rectangle로 고정되어 있다.
- 실제 footprint polygon이 더 복잡하면 일반 polygon rasterization으로 바꿔야 한다.

### `footprintFitsAt(...)`
역할:
- 특정 `(cell, heading)`에서 footprint 충돌이 없는지 검사한다.

왜 이렇게 했는가:
- 원형 로봇이 아니기 때문이다.
- clearance threshold 하나로는 직사각형 로봇의 실제 충돌 여부를 판단할 수 없다.

중요:
- grid 밖으로 나가면 바로 false다.
- obstacle 셀을 밟으면 false다.
- `unknown_is_blocked_for_feasibility`가 true면 unknown도 false다.

아쉬운 점:
- 현재는 kinematic reachability는 보지 않는다.
- 오직 collision-feasible pose만 본다.

### `computeHeadingFeasibleMasks()`
역할:
- 각 heading bin마다 feasible map을 생성한다.

왜 이렇게 했는가:
- 지금 로봇은 제자리 회전도 어렵고 footprint 방향 의존성이 크기 때문이다.
- 따라서 feasible은 단일 2D map이 아니라 heading별 pose feasibility여야 한다.

아쉬운 점:
- 계산량이 `cell_count * heading_count * stencil_size` 수준으로 꽤 크다.
- 맵이 커지거나 heading bin이 많아지면 병목 가능성이 있다.

추후 개선 후보:
- obstacle 주변만 검사
- free connected region 기반 pruning
- SIMD 또는 병렬화

---

## 현재 코드에서 먼저 확인하면 좋은 아쉬운 지점

새 노드를 붙이기 전에 아래를 먼저 확인하는 것이 좋다.

1. `origin 변경 시 evidence를 버리는 정책`
- 지금은 맞는 선택이지만, local rolling map을 원하면 부족하다.

2. `classifyCells()`의 hysteresis 문서성
- 동작은 맞는데 코드 가독성은 약간 아쉽다.

3. `clearance`의 정확도
- 지금은 충분히 실용적이지만 exact EDT가 필요할 수도 있다.

4. `feasibility` 계산량
- heading 수와 footprint 크기가 커지면 비용이 커진다.

5. `free-space ray tracing 부재`
- 지금은 point hit만 사용한다.
- lidar 빈 공간 정보는 아직 활용하지 않는다.

6. `interaction pose layer 부재`
- 현재 코어는 obstacle/clearance/feasible까지만 있다.
- 실제 목표 물체 상호작용 단계는 아직 다음 단계다.

---

## 결론
현재 코드는 "노드 붙이기 전 검증 가능한 코어 라이브러리"로는 구조가 나쁘지 않다.

특히 좋은 점:
- 의존성이 적다.
- persistent evidence 구조가 분명하다.
- heading-aware feasibility가 이미 들어가 있다.

다만 다음 단계 전에 꼭 볼 부분은:
- evidence threshold 튜닝
- feasible 계산 비용
- origin policy
- interaction pose 확장 방향

즉 지금은 바로 노드부터 붙이기보다,
이 문서 기준으로 코어 동작을 먼저 검증하는 접근이 맞다.
