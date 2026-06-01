# Configuration Guide

이 문서는 현재 프로젝트에서 직접 조정하는 YAML 설정을 설명합니다. 거리 단위는 기본적으로 m, 각도 단위는 rad, 시간 단위는 sec입니다.

## `approach_map_runner/config/runner_config.yaml`

`approach_map_runner_node`가 입력 cloud를 장애물 맵과 feasible 맵으로 변환할 때 사용하는 ROS 파라미터입니다.

### Topic 및 frame

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `input_cloud_topic` | `/approach/aligned_cloud` | mapping에 입력할 LiDAR point cloud topic입니다. |
| `target_point_topic` | `/approach/target_point` | 현재 접근 대상 좌표를 발행하는 topic입니다. |
| `grasp_targets_topic` | `/approach/grasp_targets` | grasp 대상 좌표 목록을 발행하는 topic입니다. |
| `mapping_service_name` | `/approach/mapping` | mapping 시작 및 갱신 요청을 받는 service 이름입니다. |
| `map_frame_id` | `map` | 맵 좌표계 frame입니다. |
| `obstacle_map_topic` | `/approach/obstacle_map` | 누적 evidence로 계산한 장애물 맵 topic입니다. |
| `nav2_obstacle_map_topic` | `/approach/nav2_obstacle_map` | Nav2에서 사용할 장애물 맵 topic입니다. |
| `feasible_map_topic` | `/approach/feasible_map` | 로봇 footprint가 들어갈 수 있는 위치를 나타내는 맵 topic입니다. |
| `nav2_feasible_map_topic` | `/approach/nav2_feasible_map` | Nav2에서 사용할 feasible 맵 topic입니다. |
| `clearance_map_topic` | `/approach/clearance_map` | 각 셀에서 가장 가까운 장애물까지의 거리를 나타내는 맵 topic입니다. |
| `transition_map_topic` | `/approach/transition_count_map` | 셀 상태 변화 횟수를 나타내는 맵 topic입니다. cost 안정성 계산에 사용됩니다. |
| `robot_filter_frame_id` | `base` | mapping 전처리에서 로봇 자체 포인트를 제거할 때 사용할 frame입니다. |

### 맵 기준점 및 출력

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `use_initial_origin` | `false` | 시작 시 `initial_origin_*`을 맵 원점으로 사용할지 결정합니다. |
| `allow_origin_updates_after_first_click` | `false` | 최초 target 지정 이후에도 target 클릭으로 맵 원점을 다시 옮길지 결정합니다. |
| `target_update_threshold_m` | `2.0` | mode 1 요청에서 이전 target과 이 거리보다 멀어졌을 때만 target을 다시 발행합니다. |
| `initial_origin_x_m` | `0.0` | 초기 맵 원점의 x 좌표입니다. |
| `initial_origin_y_m` | `0.0` | 초기 맵 원점의 y 좌표입니다. |
| `publish_heading_bin` | `0` | 여러 방향별 feasible 결과 중 발행할 heading bin 번호입니다. |
| `clearance_display_cap_m` | `1.5` | 시각화용 clearance 맵에서 최댓값으로 표시할 거리입니다. |
| `transform_timeout_sec` | `0.1` | point cloud TF 변환을 기다리는 최대 시간입니다. |

### Mapping 전처리

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `ground_z_min_m` | `-0.30` | ground 관측으로 누적할 z 범위의 최솟값입니다. |
| `ground_z_max_m` | `-0.20` | ground 관측으로 누적할 z 범위의 최댓값입니다. |
| `obstacle_z_min_m` | `-0.11` | obstacle 관측으로 누적할 z 범위의 최솟값입니다. |
| `obstacle_z_max_m` | `1.80` | obstacle 관측으로 누적할 z 범위의 최댓값입니다. |
| `preprocess_remove_nan_enable` | `true` | NaN 포인트 제거 여부입니다. |
| `preprocess_downsample_enable` | `true` | mapping 직전 voxel downsampling 적용 여부입니다. |
| `preprocess_outlier_removal_enable` | `true` | mapping 직전 통계 기반 outlier 제거 적용 여부입니다. |
| `preprocess_voxel_leaf_size` | `0.07` | mapping 입력 cloud의 voxel 크기입니다. 크게 하면 빨라지지만 세부 형상이 줄어듭니다. |
| `outlier_mean_k` | `20` | 통계 기반 outlier 제거에서 비교할 주변 포인트 수입니다. |
| `outlier_stddev_mul_thresh` | `1.0` | 통계 기반 outlier 제거 허용 편차 배수입니다. 작게 하면 제거가 강해집니다. |
| `robot_filter_enable` | `true` | 로봇 본체 영역의 포인트 제거 여부입니다. |
| `robot_filter_x_min` | `-0.30` | 제거할 로봇 본체 영역의 x 최솟값입니다. |
| `robot_filter_x_max` | `0.50` | 제거할 로봇 본체 영역의 x 최댓값입니다. |
| `robot_filter_y_min` | `-0.50` | 제거할 로봇 본체 영역의 y 최솟값입니다. |
| `robot_filter_y_max` | `0.50` | 제거할 로봇 본체 영역의 y 최댓값입니다. |

## `approach_map_runner/config/cost_runner_config.yaml`

`approach_cost_runner_node`가 feasible 후보 중 best arrow를 선택하고, 최종 발행 여부를 판단할 때 사용하는 ROS 파라미터입니다.

### Topic 및 frame

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `feasible_map_topic` | `/approach/feasible_map` | cost 계산의 후보 셀을 받아오는 topic입니다. |
| `transition_map_topic` | `/approach/transition_count_map` | 후보 안정성 cost 계산에 사용할 상태 변화 맵 topic입니다. |
| `target_point_topic` | `/approach/target_point` | 접근 대상 좌표 topic입니다. |
| `grasp_targets_topic` | `/approach/grasp_targets` | grasp 대상 좌표 목록 topic입니다. 값이 있으면 grasp 반경 조건으로 후보를 추가 필터링합니다. |
| `grasp_status_topic` | `/approach/grasp_status` | grasp 대상의 도달 가능 여부를 발행하는 topic입니다. |
| `robot_frame_id` | `base` | 로봇 현재 위치를 TF로 조회할 frame입니다. |
| `output_topic` | `/approach/final_cost_map` | 정규화된 최종 cost map topic입니다. |
| `arrow_topic` | `/approach/best_cost_arrow` | readiness 조건을 모두 만족했을 때만 발행되는 최종 접근 pose topic입니다. |
| `candidate_arrow_topic` | `/approach/candidate_cost_arrow` | readiness와 무관하게 현재 best 후보를 계속 표시하는 pose topic입니다. |
| `approach_ready_topic` | `/approach/approach_ready` | 최종 접근 가능 여부를 발행하는 `Bool` topic입니다. |
| `transform_timeout_sec` | `0.1` | target과 로봇 위치 TF 조회를 기다리는 최대 시간입니다. |

### 후보 필터 및 readiness

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `grasp_radius_m` | `0.9` | grasp target이 있을 때 후보 셀이 grasp 대상에서 떨어질 수 있는 최대 거리입니다. |
| `min_feasible_cells` | `25` | 최종 arrow 발행 전에 확보해야 하는 전체 feasible 후보 셀의 최소 개수입니다. |
| `best_neighbor_radius_m` | `0.30` | best 후보 주변 밀도를 검사할 반경입니다. |
| `min_best_neighbor_cells` | `5` | best 후보 주변 반경 안에 있어야 하는 feasible 셀의 최소 개수입니다. |
| `stable_duration_sec` | `0.70` | best 후보가 충분히 안정적이어야 하는 최소 지속 시간입니다. |
| `stable_position_tolerance_m` | `0.15` | 안정화 시간 동안 best 후보가 움직일 수 있는 허용 반경입니다. |

`arrow_topic`은 전체 후보 수, best 주변 후보 수, 위치 안정화 조건을 모두 통과한 경우에만 발행됩니다. 조건 통과 전 후보 위치를 확인하려면 `candidate_arrow_topic`을 사용합니다.

### 디버그 저장

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `debug_save_enable` | `true` | feasible map, cost map, CSV 로그 저장 여부입니다. |
| `debug_output_dir` | `/home/nvidia/inha_log/module/approach_cost_debug` | 디버그 파일을 저장할 상위 경로입니다. |
| `debug_save_every_n_frames` | `20` | 몇 callback마다 이미지 파일을 저장할지 결정합니다. CSV에는 callback별 정보가 기록됩니다. |
| `debug_max_frames_per_session` | `600` | target session 하나에서 저장할 최대 callback 수입니다. `0`이면 제한하지 않습니다. |

## `approach_map_runner/config/icp_config.yaml`

`node_icp_cuda`가 원본 LiDAR cloud를 정합하고 누적 cloud를 생성할 때 사용하는 ROS 파라미터입니다.

### 공통, frame 및 topic

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `measure_registration_metrics` | `false` | ICP 처리 시간과 메모리 측정 로그 활성화 여부입니다. |
| `frames.map` | `map` | ICP 결과를 누적할 맵 frame입니다. |
| `frames.odom` | `odom` | ICP initial guess 계산에 사용할 odometry frame입니다. |
| `frames.base` | `base` | 로봇 기준 frame입니다. |
| `frames.sensor` | `livox_lidar` | 입력 LiDAR 센서 frame입니다. |
| `frames.camera` | `camera_head_link` | 카메라 frame 예약값입니다. 현재 ICP 정합 루프에서는 사용하지 않습니다. |
| `topics.input_cloud` | `/livox/lidar` | 원본 LiDAR point cloud topic입니다. |
| `topics.aligned_cloud` | `/approach/aligned_cloud` | 현재 frame의 정합 완료 cloud topic입니다. |
| `topics.accumulated_cloud` | `/approach/accumulated_cloud` | 누적 cloud topic입니다. |
| `topics.submap_cloud` | `/approach/submap_cloud` | ICP target으로 사용하는 최근 submap cloud topic입니다. |
| `services.accumulation` | `accumulate` | 누적 시작 및 정지 service 이름입니다. 시작 요청의 `hz`가 실제 처리 주기를 결정합니다. |

### ICP registration

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `registration.backend` | `fast_gicp` | 정합 backend입니다. `pcl_icp`, `pcl_gicp`, `fast_gicp`, `fast_vgicp` 등을 선택할 수 있습니다. |
| `registration.num_threads` | `4` | CPU 기반 정합에서 사용할 thread 수입니다. |
| `registration.resolution` | `0.25` | voxel 기반 VGICP 및 CUDA backend에서 사용할 정합 voxel 해상도입니다. |
| `registration.max_correspondence_distance` | `1.0` | 대응점으로 인정할 최대 거리입니다. |
| `registration.transformation_epsilon` | `1.0e-4` | 변환량 기반 수렴 판정 임계값입니다. |
| `registration.euclidean_fitness_epsilon` | `1.0e-3` | fitness 변화량 기반 수렴 판정 임계값입니다. |
| `registration.max_iterations` | `100` | ICP 반복 횟수 상한입니다. |
| `registration.max_submap_frames` | `8` | ICP target submap에 유지할 최근 정합 frame 수입니다. |
| `registration.max_angular_velocity_for_update` | `0.35` | 이 회전 속도를 초과한 frame은 왜곡 가능성이 높다고 보고 정합에서 제외합니다. |
| `registration.max_planar_translation_correction` | `1.0` | initial guess 대비 ICP 보정 결과에서 허용할 xy 이동량 상한입니다. |
| `registration.max_yaw_correction` | `0.7` | initial guess 대비 허용할 yaw 보정량 상한입니다. |
| `registration.max_roll_correction` | `0.15` | initial guess 대비 허용할 roll 보정량 상한입니다. |
| `registration.max_pitch_correction` | `0.15` | initial guess 대비 허용할 pitch 보정량 상한입니다. |
| `registration.max_z_correction` | `0.25` | initial guess 대비 허용할 z 보정량 상한입니다. |

### ICP 전처리

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `preprocess.remove_nan_enable` | `true` | NaN 포인트 제거 여부입니다. |
| `preprocess.downsample_enable` | `true` | voxel downsampling 적용 여부입니다. |
| `preprocess.outlier_removal_enable` | `true` | 통계 기반 outlier 제거 적용 여부입니다. |
| `preprocess.radius_outlier_removal_enable` | `true` | 반경 기반 outlier 제거 적용 여부입니다. |
| `preprocess.robot_filter_enable` | `true` | 로봇 본체 영역 포인트 제거 여부입니다. |
| `preprocess.ground_removal_enable` | `true` | ICP registration cloud에서 ground 포인트를 제거할지 결정합니다. mapping cloud에는 ground가 유지됩니다. |
| `preprocess.voxel_leaf_size` | `0.05` | mapping 및 누적 cloud에 적용할 voxel 크기입니다. 작게 하면 맵이 촘촘하지만 느려집니다. |
| `preprocess.registration_voxel_leaf_size` | `0.07` | ICP registration 전용 voxel 크기입니다. mapping용 값보다 크게 설정하면 ICP가 가벼워집니다. |
| `preprocess.passthrough_robot_x_min` | `-0.30` | 제거할 로봇 본체 영역의 x 최솟값입니다. |
| `preprocess.passthrough_robot_x_max` | `0.60` | 제거할 로봇 본체 영역의 x 최댓값입니다. |
| `preprocess.passthrough_robot_y_min` | `-0.6` | 제거할 로봇 본체 영역의 y 최솟값입니다. |
| `preprocess.passthrough_robot_y_max` | `0.6` | 제거할 로봇 본체 영역의 y 최댓값입니다. |
| `preprocess.passthrough_ground_z` | `0.10` | ICP registration cloud에서 유지할 z 최솟값입니다. |
| `preprocess.mean_k` | `20` | 통계 기반 outlier 제거에서 비교할 주변 포인트 수입니다. |
| `preprocess.stddev_mul_thresh` | `1.0` | 통계 기반 outlier 제거 허용 편차 배수입니다. 작게 하면 제거가 강해집니다. |
| `preprocess.radius_search_m` | `0.15` | 반경 기반 outlier 제거에서 이웃을 검색할 거리입니다. |
| `preprocess.min_neighbors_in_radius` | `3` | 검색 반경 안에 필요한 최소 이웃 포인트 수입니다. |

## `approach_mapping/config/map_config.yaml`

맵 크기, evidence 누적 방식, footprint 기반 feasibility 판정을 설정합니다.

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `resolution_m` | `0.10` | 맵 셀 한 칸의 크기입니다. |
| `width_m` | `5.0` | 맵 전체 x 방향 길이입니다. |
| `height_m` | `5.0` | 맵 전체 y 방향 길이입니다. |
| `obstacle_hit_cap_per_update` | `3` | 한 update에서 셀 하나에 반영할 obstacle hit 수 상한입니다. |
| `ground_hit_cap_per_update` | `3` | 한 update에서 셀 하나에 반영할 ground hit 수 상한입니다. |
| `obstacle_weight` | `2` | obstacle hit 하나가 evidence score를 올리는 양입니다. |
| `ground_weight` | `1` | ground hit 하나가 evidence score를 내리는 양입니다. |
| `evidence_clip_value` | `100` | 누적 evidence score 절댓값 상한입니다. |
| `occupied_score_threshold` | `6` | evidence score가 이 값 이상이면 obstacle 셀로 판정합니다. |
| `free_score_threshold` | `4` | evidence score가 이 값의 음수 이하이면 free 셀로 판정합니다. |
| `footprint_length_m` | `0.70` | feasible 판정에 사용할 로봇 footprint 길이입니다. |
| `footprint_width_m` | `0.60` | feasible 판정에 사용할 로봇 footprint 폭입니다. |
| `footprint_margin_m` | `0.05` | footprint 외곽에 추가할 안전 여유 거리입니다. |
| `heading_bin_count` | `16` | 360도를 나누어 feasible 여부를 계산할 방향 개수입니다. |
| `unknown_is_blocked_for_feasibility` | `true` | 아직 관측되지 않은 셀을 feasible 판정에서 막힌 공간으로 취급할지 결정합니다. |

셀의 누적 evidence score는 아래 방식으로 갱신됩니다.

```text
delta = obstacle_weight * min(obstacle_hits, obstacle_hit_cap_per_update)
      - ground_weight   * min(ground_hits,   ground_hit_cap_per_update)
```

## `approach_cost/config/cost_config.yaml`

feasible 후보들의 최종 cost 조합 방식을 설정합니다. 각 cost layer는 후보 셀 안에서 `0.0`부터 `1.0` 사이로 정규화되며 낮을수록 우선순위가 높습니다.

| 파라미터 | 현재 값 | 의미 |
| --- | --- | --- |
| `mode` | `1` | 최종 cost 계산 방식입니다. 현재 cost 모듈은 mode 1만 지원합니다. |
| `common.weight_target_distance` | `1.0` | target까지 거리가 가까운 후보를 선호하는 가중치입니다. |
| `common.weight_stability` | `1.0` | 주변 셀 상태 변화가 적은 후보를 선호하는 가중치입니다. |
| `common.weight_approach_distance` | `1.0` | 현재 로봇 위치에서 가까운 후보를 선호하는 가중치입니다. |
| `common.stability_window_radius_cells` | `1` | 안정성 cost를 계산할 때 주변 상태 변화량을 평균 낼 셀 반경입니다. |
| `common.invalid_value` | `-1.0` | cost 계산 대상이 아닌 셀에 기록할 값입니다. |
