# ur3_4040_urdf 자율 주행 매핑 디버깅 보고서

작성일: 2026-05-02
대상 패키지: `ur3_4040_urdf`, `m-explore-ros2 (explore_lite)`
환경: ROS 2 Jazzy / Gazebo (gz sim) / `ROS_DOMAIN_ID=28`

---

## 1. 문제 상황
`ur3_4040_urdf` 패키지의 `map_building.launch.py` 실행 시 SLAM Toolbox + Nav2 + explore_lite 스택은 정상적으로 기동되었으나, 로봇이 자율 탐험을 시작하지 못하고 제자리에 정지하는 현상이 발생.

## 2. 진단 절차 및 발견된 원인 (단계별)

### 2.1 노드 활성화 점검
- `ros2 node list`, `ros2 lifecycle get`, `ros2 service call /lifecycle_manager_navigation/is_active` 등으로 모든 Nav2 노드(`controller_server`, `planner_server`, `bt_navigator` 등)가 `active [3]` 상태임을 확인.
- launch 파일의 lifecycle activate 호출에서 service 타입이 `nav2_lifecycle_manager/ManageLifecycleNodes`로 잘못 기재되어 있었음 → 정상 타입은 `nav2_msgs/srv/ManageLifecycleNodes`.

### 2.2 explore_lite 데이터 흐름 점검
- `ros2 topic echo /explore/status` 결과 `exploration_started`로 노드 자체는 살아있음을 확인.
- `/explore/frontiers` topic에 데이터가 발행되지 않음 → explore_node가 frontier를 발견하지 못하는 상태.
- 원인: SLAM Toolbox의 `/map` topic 발행 주기가 5초 이상으로 길고 explore_lite의 subscription `Durability`가 `VOLATILE`이라 latched 메시지를 못 받는 구조.

### 2.3 costmap 토픽 변경 시도와 회귀
- `costmap_topic`을 `/global_costmap/costmap`으로 임시 변경했으나, nav2 global_costmap은 inflation_layer를 거친 결과물이라 frontier가 inflation 영역 안쪽에 위치해 BFS가 unknown 경계에 도달 불가 → `No frontiers found` 반복.
- 결국 SLAM의 raw `/map`(free / occupied / unknown 그대로)을 다시 사용하도록 되돌림.

### 2.4 SLAM Toolbox 매핑 확장 부진
- `minimum_travel_distance: 0.5`, `minimum_travel_heading: 0.5 rad (~28°)` → 작은 회전·이동이 새 keyframe으로 등록되지 않아 매핑 영역이 robot 주변 1~2m 수준에 정체.
- `map_update_interval: 5.0`도 explore의 첫 메시지 수신을 어렵게 만듦.

### 2.5 nav2 planner / controller 단의 실패 분석
`/behavior_tree_log` 및 `/rosout` 캡처를 통해 실제 실패 메시지 확보:

- `GridBased plugin failed to plan from (...) to (...): "Failed to create plan with tolerance of: 0.500000"`
- `[compute_path_to_pose] [ActionServer] Aborting handle.`
- `Reached the goal!` 메시지가 0.001초 내에 발생 → robot이 안 움직였음에도 즉시 도달 처리.

원인 다중:
- `GridBased.use_astar: false` → Dijkstra가 큰 grid에서 매우 느림 (`expected_planner_frequency: 20Hz`를 못 맞춤).
- `inflation_radius: 0.7m` → 좁은 free space 환경에서 robot 위치 자체가 lethal cost로 잡혀 plan 시작점 invalid.
- `controller_frequency: 20Hz` + `MPPI batch_size: 2000`, `time_steps: 56` → CPU 환경에서 controller가 cmd_vel 발행 주기를 못 맞춤.
- `progress_checker: 0.5m / 10s` → controller가 명령을 못 내는 동안 progress 검증 실패로 abort.
- `general_goal_checker.xy_goal_tolerance: 0.25` + `GridBased.tolerance: 1.5` → plan path가 매우 짧게 만들어져 controller가 즉시 reach 판정.

### 2.6 collision_monitor의 cmd_vel 차단
- `/rosout`에서 `collision_monitor: "Robot to approach for 1.20 seconds away from collision"` 로그 확인.
- sparse한 LiDAR(1°당 1빔)로 인해 별 모양의 occupied가 누적되며 footprint approach 폴리곤이 충돌 임박으로 오감지 → cmd_vel을 0으로 차단.

### 2.7 ROS-Gazebo 브리지 검증
- `ros_gz_bridge`의 cmd_vel 매핑(`/cmd_vel ↔ /model/L_AND_F/cmd_vel`)과 Gazebo plugin 동작을 `gz topic` 직접 명령으로 검증하여 ROS → Bridge → Gazebo 경로 자체는 정상임을 확인 (직접 명령 시 `odom.twist.linear.x ≈ 0.2` 확보).

## 3. 적용한 변경 사항

### 3.1 영구 반영 (파일 수정)

**`src/ur3_4040_urdf/launch/map_building.launch.py`**

- 잘못된 lifecycle service 타입 수정: `nav2_lifecycle_manager/ManageLifecycleNodes` → `nav2_msgs/srv/ManageLifecycleNodes`, command `1` → `0` (STARTUP)
- AMCL 미사용으로 무효였던 `/initialpose` publish 액션(`set_initial_pose`) 제거.
- launch 시작 후 `tune_params` (TimerAction, period=10s) 추가 — Nav2/SLAM 파라미터를 한 번에 자동 적용:

| 노드 | 파라미터 | 변경 전 → 변경 후 |
|---|---|---|
| global_costmap | inflation_layer.inflation_radius | 0.7 → 0.1 |
| local_costmap | inflation_layer.inflation_radius | 0.7 → 0.1 |
| planner_server | GridBased.use_astar | false → true |
| planner_server | GridBased.tolerance | 0.5 (default) → 0.5 (명시) |
| controller_server | FollowPath.batch_size | 2000 → 500 |
| controller_server | FollowPath.time_steps | 56 → 30 |
| controller_server | progress_checker.movement_time_allowance | 10.0 → 30.0 |
| controller_server | progress_checker.required_movement_radius | 0.5 → 0.1 |
| controller_server | general_goal_checker.xy_goal_tolerance | 0.25 → 0.1 |
| controller_server | general_goal_checker.yaw_goal_tolerance | 0.25 → 3.14 |
| collision_monitor | FootprintApproach.enabled | true → false |
| slam_toolbox | map_update_interval | 5.0 → 1.0 |
| slam_toolbox | minimum_travel_distance | 0.5 → 0.1 |
| slam_toolbox | minimum_travel_heading | 0.5 → 0.1 |

- `explore_launch` 시작 시점을 10초 → 15초로 늦춰 `tune_params` 적용 후 explore가 시작되도록 조정.

**`src/m-explore-ros2/explore/config/params.yaml`** (및 install 측 동기 사본)

- `min_frontier_size`: 0.5(default) → 0.75
  (디버깅 과정에서 0.5 → 0.2 → 0.4 → 0.75로 단계 조정)
- 그 외 `costmap_topic: map`, `costmap_updates_topic: map_updates` 등 명시적으로 정렬 (explore가 SLAM raw map을 직접 보도록).

`install/` 폴더의 launch / yaml에도 동일 내용을 동기화하여 `colcon build` 없이도 다음 launch 실행 시 자동 반영되도록 함.

## 4. 검증 결과

영구 반영 후 launch 재실행만으로 다음 로그 흐름 확인:

- `bt_navigator: Begin navigating from current location (9.83, -0.02) to (9.02, -1.70)`
- 이후 `controller_server: Received a goal, begin computing control effort.` 정상 진행.
- `/odom`의 `twist.linear.x ≈ 0.5 m/s` (vx_max에 도달, 실제 직진 중).

explore_node가 frontier를 발행하고 nav2가 path를 만들고 controller가 cmd_vel을 publish하여 로봇이 자율 탐험하는 사이클 정상 동작.

## 5. 결론 및 후속 권장 사항

주요 원인은 다음의 복합이었음:

1. sparse LiDAR / 좁은 매핑 영역과 nav2 default 파라미터의 부조화
2. MPPI + 20Hz 조합의 CPU 부하 한계
3. collision_monitor의 footprint approach 오감지

추가 안정화를 원할 경우 다음을 고려:

- 시스템 default가 아닌 패키지 자체 `nav2_params.yaml`, `slam_params.yaml`을 작성하여 launch에서 `params_file` 인자로 명시 전달 → 현재의 `tune_params`(런타임 `ros2 param set` 묶음)을 yaml 기반으로 깔끔하게 대체.
- `controller_frequency`는 동적 변경이 불가하므로 위 자체 yaml에서 5~10Hz로 명시 설정 권장.
- `obstacle_layer`의 `raytrace_max_range`, `obstacle_max_range`를 LiDAR 사양에 맞춰 명시하여 별 모양 누적 occupied 현상 완화.

---

## 6. 추가 발생 이슈 및 후속 조치

### 6.1 SLAM 누적 drift로 인한 map 왜곡
- 자율 탐험을 한참 진행한 뒤 RViz `/map`에서 직각이어야 할 벽이 비스듬히 휘는 현상 관찰.
- 원인: 빠른 직진/회전 속도와 너무 작은 keyframe 임계값으로 인해 scan-to-scan match가 실패하면서 누적 drift가 발생, loop closure 가능 임계를 넘어선 것으로 판단.
- 결과적으로 explore_lite는 free↔unknown 경계가 끊긴 상태에서 frontier를 더 이상 발견하지 못하고 `No frontiers found` → `Returning to initial pose`로 자체 종료.
- `[tf2_buffer]: Detected jump back in time. Clearing TF buffer.` 경고도 한 번 발생 (sim 시간 점프 흔적).

### 6.2 안정화 파라미터 추가 반영
launch 파일의 `tune_params`에 다음을 추가하여 매핑 안정성을 높임:

| 노드 | 파라미터 | 변경 |
|---|---|---|
| controller_server | FollowPath.vx_max | 0.5 → 0.3 (천천히 이동) |
| controller_server | FollowPath.wz_max | 1.9 → 1.0 (회전 시 scan match 안정성↑) |
| slam_toolbox | minimum_travel_distance | 0.1 → 0.25 (작은 noise scan을 keyframe에서 제외) |
| slam_toolbox | minimum_travel_heading | 0.1 → 0.2 |

### 6.3 launch 재실행 시 explore_node 중복 발생
- 이전 launch 인스턴스가 완전히 종료되지 않은 상태에서 새 launch가 실행되면, `ros2 node list` 결과에 `/explore_node`가 두 번 나타나며 `WARNING: there are nodes in the graph that share an exact name` 메시지가 출력됨.
- 두 인스턴스가 동시에 `/navigate_to_pose` 액션 클라이언트로 goal을 보내면서 nav2가 혼란을 겪을 수 있어, 이전 PID를 `kill`로 정리하여 단일 인스턴스만 남김.
- 권장 운용: launch 재실행 전 `pgrep -af explore_lite/lib/explore_lite/explore` 등으로 잔존 프로세스를 확인하고 종료할 것.

### 6.4 재가동 검증
- `tune_params` 적용 후 explore_node 중복 정리한 시점에 odometry는 `linear.x ≈ 0.227 m/s`로 측정되어 robot이 정상적으로 이동 중임을 확인.
- explore_node는 `Begin navigating from (3.32, -1.51) to (5.59, -2.75)` 형태로 새 frontier goal을 지속 발행. 이전보다 vx_max가 낮아 시각적으로는 느려 보일 수 있으나, 매핑 누적 오차가 줄어 안정성은 향상됨.

---

## 7. 충돌로 인한 odom drift 및 footprint 미스매치 대응

### 7.1 관찰된 현상
- 안정화 파라미터 적용 후에도 한참 운용한 뒤 RViz `/map`이 다시 어긋나는 현상 발생.
- 사용자 관찰: 로봇이 장애물에 부딪힌 뒤 wheel은 계속 회전하고, 이어 회전하다가 **뒷부분이 벽에 걸리는** 상황.

### 7.2 원인 분석
1. **충돌 시 wheel slip → odom 발산**
   - Gazebo의 differential drive plugin은 wheel rotation을 그대로 적분해 `/odom`을 발행함.
   - 로봇이 벽에 박힌 상태에서도 wheel은 헛도므로 odom 상에서는 계속 이동한 것처럼 누적되고, SLAM Toolbox가 이 잘못된 odom을 prior로 사용해 scan match가 한쪽으로 끌려가 map이 어긋남.
2. **회전 시 robot 모서리가 벽에 sweep**
   - 운영 중 `/global_costmap/global_costmap robot_radius`가 **0.22m**로 설정되어 있음을 확인.
   - 실제 로봇 모델은 `mobile_base.xacro`상 wheel이 base 중심에서 y=0.271m 떨어져 있어 가로폭만 ≈54cm. `ur3_4040` 이름과 형태를 종합하면 외접원 반경이 0.30~0.35m 수준.
   - 즉 nav2 입장에서는 "충분한 여유"라고 판단해 path를 만들지만, 실제 직사각형 로봇이 회전하면 모서리가 외접원 반경까지 sweep되어 벽에 닿음 → 충돌 → 7.2.1의 odom 발산으로 연쇄.

### 7.3 적용한 추가 변경 사항 (`tune_params`에 반영)

#### 7.3.1 후진 봉쇄
| 노드 | 파라미터 | 변경 |
|---|---|---|
| controller_server | FollowPath.vx_min | -0.35 → 0.0 |

- MPPI controller가 후진 trajectory를 후보군에서 제외 → path follow 도중에는 후진 미발생.
- nav2 BT의 recovery 단계에서 호출되는 `BackUp` 액션은 controller가 아닌 `behavior_server`가 직접 실행하므로 vx_min과 무관. 이 경로까지 차단하려면 BT XML의 `<BackUp>` 노드를 제거하고 `bt_navigator`의 `default_nav_to_pose_bt_xml`로 패키지 내부 BT 파일을 가리켜야 함(현 단계에선 적용 보류).

#### 7.3.2 footprint / inflation 보강
| 노드 | 파라미터 | 변경 전 → 변경 후 |
|---|---|---|
| global_costmap | robot_radius | 0.22 → 0.35 |
| local_costmap | robot_radius | 0.22 → 0.35 |
| global_costmap | inflation_layer.inflation_radius | 0.10 → 0.25 |
| local_costmap | inflation_layer.inflation_radius | 0.10 → 0.25 |
| global_costmap | footprint_padding | 0.01 → 0.05 |
| local_costmap | footprint_padding | 0.01 → 0.05 |

이 조합으로 nav2가 만드는 path가 벽으로부터
`robot_radius(0.35) + footprint_padding(0.05) + inflation_radius(0.25) ≈ 0.65 m`
의 여유를 확보하게 됨 → 회전 sweep 시에도 모서리가 벽에 닿지 않을 충분한 거리.

### 7.4 한계 및 향후 개선 방향
- 현재는 `robot_radius` 기반 원형 footprint 가정. 실제 로봇이 직사각형이므로 정확한 다각형 footprint를 `footprint` 파라미터에 명시하면 더 효율적인 path가 생성되며 좁은 통로 통과도 개선됨.
  - 예: `footprint: "[[0.27,0.20],[0.27,-0.20],[-0.27,-0.20],[-0.27,0.20]]"`
- 충돌 시 odom 발산 문제는 근본적으로 wheel encoder 단독 odom의 한계. `robot_localization` 패키지의 `ekf_node`로 wheel odom과 이미 발행 중인 `/imu`를 EKF 융합하면 충돌 시 가속도가 0임이 IMU에 잡혀 odom drift가 크게 줄어듦.
- Gazebo 모델 wheel link의 `mu1`, `mu2`(정지/운동 마찰)를 강화하면 시뮬레이션 자체에서 wheel slip이 줄어들지만 정상 주행 시 회전 거동에 영향이 갈 수 있어 trade-off 존재.

---

## 8. warehouse_world 전체 매핑 성공 (2026-05-02 21:30)

### 8.1 잔존 증상
앞 7장까지 적용 후에도 다음 두 가지 증상으로 매핑이 중단되거나 일부 영역만 매핑됨.

1. **explore_lite 조기 종료** — 부트스트랩 회전 후 SW 격리구역 frontier 몇 개 처리하고는 `[explore_node] All frontiers traversed/tried out, stopping` → `Returning to initial pose` → 정지. 사용자 표현: "한바퀴 돌고 가만히 있음".
2. **중앙 게이트 통과 실패** — warehouse 중앙 vertical wall 사이 2 m 폭 게이트(`x∈[-2,0], y∈[-1,1]`)에 가까이 가면 `[planner_server] Failed to create plan with tolerance of: 0.250000` → recovery `spin` → `[behavior_server] Collision Ahead - Exiting Spin` → 같은 자리에서 회전만 반복.

### 8.2 원인 분석

#### 8.2.1 explore_lite 조기 종료
- `min_frontier_size: 0.3` — SW 격리구역 다 매핑되면 게이트 너머 unknown 경계 frontier는 자잘하게 쪼개져 0.3 m 이상이 거의 없음 → 모두 필터링 → "No frontiers".
- `return_to_init: true` — 일시적으로 frontier 비면 원점 복귀 후 노드 종료. 사용자 입장에서 robot이 "가만히" 있는 것으로 보임.
- `progress_timeout: 30.0` — 좁은 통로 plan/통과에 30초는 부족. 한 번 시도 실패하면 해당 frontier가 영구 blacklist 되어 재시도 안 됨.
- `potential_scale: 3.0`, `gain_scale: 1.0` — 거리 패널티가 frontier 크기 보상보다 강함 → 게이트 너머의 큰 unknown space frontier가 가까운 작은 frontier에 우선순위 밀림.

#### 8.2.2 게이트 plan 실패 + spin recovery 실패
- 2 m 게이트의 자유공간: `2 - 2*inscribed(0.20) = 1.6 m`. 이론상 통과 가능.
- 그러나 `inflation_radius: 0.30` 으로 게이트 코너(-2,0) / (0,-1) / (0,1) 주변 cost가 높게 깔림 → planner의 `tolerance: 0.25` 안에서 lethal-free 셀 못 찾음 → plan fail.
- nav2 BT가 recovery로 spin(90°) 호출 → `behavior_server`가 spin trajectory의 각 angle에서 collision 검사. `consider_footprint: false` 인 상태라 inflation cost로 판단하는데 좁은 곳에서는 항상 high cost → "Collision Ahead" 로 spin 중단.
- 결과적으로 plan 실패 → spin 실패 → 같은 위치에서 최소 회전만 반복, 전진 못 함.

### 8.3 해결책

#### 8.3.1 `m-explore-ros2/explore/config/params.yaml` 변경
| 파라미터 | 변경 전 → 변경 후 | 이유 |
|---|---|---|
| `return_to_init` | `true` → **`false`** | frontier 일시 빌 때 원점 복귀/종료 방지. 새 frontier 생기면 즉시 재탐색 |
| `min_frontier_size` | `0.3` → **`0.1`** | 게이트 너머 작은 frontier도 잡아 unknown 경계 외삽 가능 |
| `progress_timeout` | `30.0` → **`60.0`** | 좁은 통로 plan/통과 시간 확보, 잘못된 blacklist 방지 |
| `planner_frequency` | `0.15` → **`0.33`** | frontier 재평가 6.7 s → 3 s. 게이트 너머 새 영역 보이면 빠르게 그쪽 잡음 |
| `potential_scale` | `3.0` → **`1.0`** | 거리 패널티 down |
| `gain_scale` | `1.0` → **`5.0`** | 큰 frontier 우대 → 게이트 너머 unknown space 우선 |

#### 8.3.2 `config/nav2_params.yaml` 변경
| 노드 | 파라미터 | 변경 전 → 변경 후 | 이유 |
|---|---|---|---|
| local_costmap | `inflation_layer.inflation_radius` | `0.30` → **`0.25`** | 게이트 통과 자유공간 확보. 0.25 > inscribed 0.20 이라 lethal 위 마진 유지 |
| global_costmap | `inflation_layer.inflation_radius` | `0.30` → **`0.25`** | 동일 |
| controller_server | `FollowPath.CostCritic.consider_footprint` | `false` → **`true`** | inflation에만 의존 안 하고 실제 40×54 cm 직사각형으로 충돌 판정 → 회전 sweep(circumscribed 0.337)까지 정확히 검사. 엉덩이 충돌/잘못된 spin abort 차단 |

#### 8.3.3 부수 효과 (무해)
- `consider_footprint: true` 켠 후 `[computeCircumscribedCost]` 가 ERROR 로그를 반복 출력:
  ```
  inflation_radius (0.250000) is smaller than circumscribed_radius (0.350000)
  ```
- 의미: nav2의 빠른 충돌 검사 trick("inflation 밖이면 무조건 안전")을 못 쓰고 매 trajectory point마다 full footprint 검사로 fallback. **결과: planning 약간 느려질 뿐 정확도/안전성 동일**. 로그 레벨이 ERROR로 과장돼 있을 뿐 실제로는 WARNING 수준.
- 경고 제거하려면 `inflation_radius`를 0.34 이상으로 올려야 하는데, 그러면 게이트 통과가 다시 어려워짐 → 현 상태 유지.

### 8.4 검증 결과
위 두 yaml 변경 + 기존 7장까지 누적 설정으로 `ros2 launch ur3_4040_urdf map_building.launch.py` 1회 실행 시:
- Robot이 SW 구역 매핑 후 게이트 (-2,0~0,1) 통과 → NW/NE/SE 구역 순차 매핑.
- 약 N분 내 warehouse_world 전체(30 × 24 m) 매핑 완료.
- 사용자 확인: "드디어 제대로 맵이 다 그려졌어".

### 8.5 핵심 교훈
- **explore_lite의 조기 종료 패턴은 단일 파라미터 문제가 아님.** `return_to_init` + `min_frontier_size` + `progress_timeout` 세 가지가 동시에 작용. 좁은 게이트가 있는 환경에서는 셋 다 완화 필요.
- **`consider_footprint: false`는 정사각/원형에 가까운 robot에서만 안전.** 종횡비가 1:1.35인 본 robot처럼 직사각형이 길쭉하면 inflation만으로는 회전 sweep을 못 가린다 → controller가 잘못된 collision 판정으로 spin 자체를 abort. 직사각형 footprint를 쓸 거면 `consider_footprint: true`가 사실상 필수.
- **inflation은 안전 마진이 아니라 cost gradient 도구.** 실제 충돌 안전은 footprint 검사가 책임짐. inflation 줄였다고 충돌 위험 직접 증가하지 않음 (단, planner가 벽에 더 붙은 path 만들 수는 있어서 controller의 footprint 검사가 받쳐줘야 함).

---

## 9. Saved map 저장 및 후처리 (2026-05-03)

### 9.1 map 저장
nav2 `map_saver_cli`로 저장. 기본 QoS는 SLAM Toolbox의 `/map`(TRANSIENT_LOCAL+RELIABLE) 와 안 맞아 spin 타임아웃 → `map_subscribe_transient_local:=true` 인자 필수.
```
ros2 run nav2_map_server map_saver_cli -f my_capstone_map \
  --ros-args -p map_subscribe_transient_local:=true
```
결과: `my_capstone_map.pgm` (600×485 픽셀, 0.05 m/cell) + `my_capstone_map.yaml`.

### 9.2 lidar self-hit 진단
저장된 map을 RViz `/map` 토픽으로 보면 robot 이동 경로 따라 동심원 형태로 검정 점들 잔존. opencv `connectedComponentsWithStats`로 확인:
- 원본 occupied 9,084 픽셀, 569개 군집
- 군집 크기 분포: min 1, median 3, max 1,460
- 작은 군집(< 20 픽셀): 550개 = stale lidar self-hit
- 큰 군집(> 100 픽셀): 15개 = 진짜 벽

원인: lidar 위치 base_link `(0.3, 0, 0.15)`, robot 4040 chassis 모서리까지 거리 0.288 ~ 0.57 m. lidar `min_range=0.35` 보다 멀어 chassis 일부가 매 scan에서 hit → SLAM이 occupied로 confirm.

### 9.3 후처리 (`scripts/clean_map.py`)
opencv connected components로 작은 군집(default < 20 픽셀)을 free(254)로 변환. 큰 군집(벽)은 보존.
```
ros2 run ur3_4040_urdf clean_map.py my_capstone_map.pgm
```
결과: 550개 군집 제거, 19개 벽 군집 보존, robot 시작점 (map 0,0) 주변 완전 free 확인.

### 9.4 재발 방지 (costmap 단)
xacro의 lidar `min_range`를 늘리면 좁은 통로 통과 능력 손상, scan 각도 좁히면 SLAM 매핑 quality 떨어짐(samples 360→300 시도 시 occupied confirm 부족). 따라서 nav2 costmap 단에서 self-hit hit을 무시하는 방식 선택:

| 노드 | 파라미터 | 변경 |
|---|---|---|
| local_costmap.voxel_layer.scan | `obstacle_min_range` | 0.0 → **0.6** |
| local_costmap.voxel_layer.scan | `raytrace_min_range` | 0.0 → **0.6** |
| global_costmap.obstacle_layer.scan | `obstacle_min_range` | 0.0 → **0.6** |
| global_costmap.obstacle_layer.scan | `raytrace_min_range` | 0.0 → **0.6** |

chassis 가장 먼 모서리(0.57 m) 너머 hit만 obstacle로 mark → live costmap에 self-hit 점 안 박힘. saved map의 wall은 static_layer로 그대로 유지.

---

## 10. Localization-only Navigation 단계

### 10.1 `launch/navigate.launch.py` 구성
| 컴포넌트 | 역할 |
|---|---|
| Gazebo + spawn + bridge | 시뮬 + ROS↔gz 토픽 변환 |
| nav2 `localization_launch.py` (5 s 후 시작) | map_server + AMCL + lifecycle_manager_localization |
| nav2 `navigation_launch.py` (12 s 후 시작) | planner/controller/bt + lifecycle_manager_navigation |
| RViz | 시각화 + 2D Goal Pose 입력 |

`bringup_launch.py`를 통째로 호출하면 lifecycle race condition 발생 → localization과 navigation을 분리 + TimerAction으로 시간차 두는 게 안정적.

### 10.2 AMCL 자동 초기 위치
nav2_params.yaml의 amcl 섹션에:
```
set_initial_pose: true
initial_pose:
  x: 0.0   # map frame (= world spawn -3,-3 의 SLAM 시작 좌표)
  y: 0.0
  z: 0.0
  yaw: 0.0
```
**주의**: AMCL `initial_pose`는 **map frame** 기준. world spawn `(-3, -3)`을 그대로 넣으면 매핑된 map 좌표계와 어긋나 lidar matching 발산.

### 10.3 ros_gz_bridge use_sim_time
gazebo.launch.py의 bridge 노드에 `parameters=[{'use_sim_time': True}]` 누락 시: bridge가 wall clock stamp로 odom→base_footprint TF를 발행, AMCL은 sim_time stamp로 map→odom 발행 → TF chain 보간 실패 → controller가 robot 위치 lookup 못 함 → cmd_vel 잘못 발행. 1줄 추가로 해결.

### 10.4 Controller 교체: MPPI → RPP
| 항목 | MPPI (변경 전) | RPP (변경 후) |
|---|---|---|
| 알고리즘 | 매 cycle 2,000 trajectory × 56 timesteps × 9 critics 평가 | lookahead 1점 + 단순 geometry |
| CPU 부담 | 무거움 (consider_footprint=true 시 full footprint 8개 모서리 매번 검사) | 가벼움 |
| 좁은 통로 | 응답 늦어 jerky 회전, recovery spin 빈발 | 안정적 |
| 결과 | "반대로 회전 + Goal failed" 반복 (실제는 plan 못 만들어 spin recovery 진입) | 직진 path 따라감 |

추가로 RPP의 `use_rotate_to_heading: true` 로 큰 회전 시 제자리 회전 우선 → 잘못된 회전 방향 차단.

### 10.5 좁은 통로 안정화
`lookahead_dist: 0.6 → 0.4`, `max_lookahead_dist: 0.9 → 0.6`. 게이트 통과 시 carrot 점이 wall 너머 안 가리키게 → 흔들림 감소.

### 10.6 Goal yaw 반영
`general_goal_checker.yaw_goal_tolerance: 3.14 → 0.25` (Pinky 동일). 사용자가 RViz Goal Pose에서 지정한 방향 yaw까지 robot이 회전한 뒤 goal 인정.

매핑 시에는 explore_lite가 frontier 도달만 신경쓰면 되므로 yaw_goal_tolerance가 작으면 blacklist 누적. `map_building.launch.py`에서 `RewrittenYaml`로 매핑 동안만 3.14로 override:
```python
from nav2_common.launch import RewrittenYaml
mapping_nav2_params = RewrittenYaml(
    source_file=nav2_params_file,
    param_rewrites={'yaw_goal_tolerance': '3.14'},
    convert_types=True,
)
```

---

## 11. Goal 명령 인터페이스

### 11.1 직접 호출 (CLI)
nav2 active 상태면 `navigate_to_pose` action이 자동 제공:
```
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'},
         pose: {position: {x: 5.0, y: 2.0, z: 0.0},
                orientation: {x: 0, y: 0, z: 0.707, w: 0.707}}}}" --feedback
```
`--feedback` 으로 거리/ETA/recovery 횟수 실시간 출력, 종료 시 SUCCEEDED/ABORTED 자동 표시.

### 11.2 다른 노드와 통합용 wrapper (`scripts/goal_relay_node.py`)
- 입력: `/simple_goal` (geometry_msgs/PoseStamped)
- 처리: `navigate_to_pose` action 호출, feedback/result 콜백으로 logger 출력
- 출력: `/navigation_result` (std_msgs/String) — `SUCCEEDED` / `ABORTED` / `CANCELED` / `REJECTED`
- 새 goal 오면 진행 중 goal 자동 cancel (preemption)

비전 노드 등 다른 application에서 PoseStamped 발행 + 결과 String 구독 형태로 통합.

### 11.3 `launch/mission.launch.py` (headless)
Gazebo + nav2 + goal_relay_node (RViz 없음). 외부에서 `/simple_goal` 발행으로 미션 실행. 결과는 launch 터미널 logger에 표시.

---

## 12. 실물 robot 이전 계획

### 12.1 시스템 토폴로지
- 노트북 (robot 위 탑재): nav2 + 비전 노드 + 미션 노드 (headless)
- 라즈베리 파이: lidar driver + IMU driver
- 노트북-Pi: 같은 ROS_DOMAIN_ID=28, Ethernet 또는 WiFi (DDS 자동 발견)

### 12.2 시뮬→실물 변경점
| 시뮬 (현재) | 실물 |
|---|---|
| `gazebo.launch.py` (gz spawn + ros_gz_bridge) | 제거 |
| gz-sim diff_drive plugin (`/odom`, `/cmd_vel`) | 모터 드라이버 노드 |
| `gpu_lidar` 시뮬 (`/scan`) | rplidar_ros 등 실제 driver |
| 시뮬 IMU | bno055 driver |
| AMCL `set_initial_pose: (0,0,0)` | `false`로 두고 매번 시작 시 정해진 도킹 위치에서 publish |

`mission.launch.py` 형태가 headless 운영에 그대로 활용 가능. Gazebo include만 빼고 lidar/imu/모터 driver launch include로 교체.

### 12.3 미션 노드 패턴
비전 노드가 발행하는 좌표(PointStamped) 구독 → goal_relay_node로 PoseStamped 전달 → result 받아서 다음 task 분기 (다음 좌표로 이동, arm 제어, 사진 촬영 등). 시뮬 단계에서 `goal_relay_node` + `/simple_goal` + `/navigation_result` 인터페이스를 동일하게 사용 가능.

