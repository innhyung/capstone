# 4040 알루미늄 프레임 모바일 매니퓰레이터 기반 SLAM 자율 주행 시스템

작성일: 2026-05-03
대상 패키지: `ur3_4040_urdf`

---

## Abstract

본 연구는 4040 알루미늄 프로파일 섀시에 4륜 차동 구동(diff-drive) 모바일 베이스와 UR3 매니퓰레이터를 결합한 로봇 모델(`L_AND_F`)을 대상으로, ROS 2 Jazzy 환경에서 SLAM Toolbox·Nav2·explore_lite 스택을 통합하여 (1) 자율 매핑, (2) 저장된 맵 기반 자율 주행, (3) 외부 토픽으로 전달된 좌표에 대한 미션 실행을 가능하게 하는 파이프라인을 구축하였다. Gazebo Harmonic 시뮬레이터의 `warehouse_world` (30 × 24 m)에서 검증을 수행하였으며, 직사각형 footprint(40 × 54 cm)와 4040 섀시 구조에서 발생하는 라이다 자가-검출(self-hit) 문제, MPPI 제어기의 연산 부하 문제, AMCL 좌표계 정합 문제 등을 진단·해결하였다.

## 1. 시스템 구성

### 1.1 하드웨어 모델
- 모바일 베이스: 4륜 차동 구동, wheel separation 0.541 m, wheel radius 0.0631 m
- 매니퓰레이터: UR3 (베이스 z = 0.35 m)
- 센서: 2D 라이다(`gpu_lidar`, 360°/360 samples, range 0.35–10 m), IMU(BNO055), wheel odometry
- 외형: 40 × 54 cm 직사각형 footprint

### 1.2 소프트웨어 스택
- ROS 2 Jazzy / Gazebo Harmonic
- SLAM: `slam_toolbox` (online async)
- Navigation: `nav2_bringup` (planner/controller/bt + AMCL)
- 자율 탐험: `m-explore-ros2/explore`
- 제어기: Regulated Pure Pursuit (RPP)

## 2. 자율 매핑 파이프라인 (`map_building.launch.py`)

Gazebo 시뮬→SLAM Toolbox→Nav2→14초 부트스트랩 회전→explore_lite 순으로 시간차 기동한다. 부트스트랩 회전은 Gazebo `gz topic` 명령으로 angular velocity 0.5 rad/s를 직접 전달하여 시작 시점의 unknown space를 해소한다. Nav2 파라미터는 `RewrittenYaml`로 매핑 전용 override(`yaw_goal_tolerance: 3.14`)를 적용하여 explore_lite가 frontier 도달만으로 success 처리되도록 한다.

좁은 통로(2 m)가 있는 환경에서 explore_lite가 조기 종료하는 패턴을 해소하기 위해 다음을 조정하였다: `return_to_init: false`, `min_frontier_size: 0.1`, `progress_timeout: 60.0`, `gain_scale: 5.0`. 또한 직사각형 footprint를 정확히 평가하도록 `CostCritic.consider_footprint: true`를 활성화하고, `inflation_radius: 0.25`로 설정하여 inscribed_radius(0.20) 위 마진을 확보하였다.

## 3. 라이다 자가-검출 문제와 후처리

저장된 맵에서 로봇 이동 경로를 따라 동심원 형태의 stale obstacle 점이 관찰되었다. 라이다 위치(`(0.3, 0, 0.15)` base_link)에서 4040 섀시 모서리까지의 거리(0.288 ~ 0.57 m)가 `min_range: 0.35`보다 멀어, 매 스캔에서 섀시 일부가 occupied로 confirm되는 현상이다. 라이다 위치 변경 또는 스캔 각도 축소는 좁은 통로 통과 능력 또는 SLAM occupancy confirm 품질을 저하시키므로 다음 두 단계로 해결하였다:

1. **저장된 맵 후처리** (`scripts/clean_map.py`): OpenCV `connectedComponentsWithStats`로 occupied 군집을 분석, 임계값 미만(default 20 픽셀) 군집을 free로 변환. 본 연구에서는 569개 군집 중 550개(작은 stale)를 제거하고 19개 벽 군집을 보존하였다.
2. **Nav2 costmap 단 차단**: `obstacle_min_range: 0.6`, `raytrace_min_range: 0.6`로 설정하여 0.6 m 이내 라이다 hit을 obstacle layer에 마킹하지 않게 하였다. 정적 맵(static_layer)은 그대로 유지된다.

## 4. 자율 주행 및 미션 인터페이스 (`navigate.launch.py`, `mission.launch.py`)

`nav2_bringup`의 `localization_launch.py`와 `navigation_launch.py`를 5 초·12 초 시간차로 분리 호출하여 lifecycle race condition을 회피한다. AMCL의 `set_initial_pose`는 매핑 시작 좌표계 기준 (0, 0, 0)으로 설정하여 자동 초기화한다. `ros_gz_bridge`에 `use_sim_time: True`를 명시하여 sim_time stamp 정합성을 보장하였다(미설정 시 wall clock과의 stamp 차이로 TF chain 보간 실패 발생).

제어기는 MPPI에서 RPP로 교체하였다. MPPI의 매 cycle 2,000개 trajectory×9 critic 평가 부담은 좁은 통로에서 cmd_vel 발행 지연을 일으켜 spin recovery에 진입하는 결과를 초래하였다. RPP는 lookahead 1점 기반의 경량 알고리즘으로, `use_rotate_to_heading: true`와 `lookahead_dist: 0.4`(좁은 통로 안정화)로 설정하여 안정적으로 path를 추종한다.

## 5. Goal 명령 인터페이스

CLI에서 `ros2 action send_goal /navigate_to_pose --feedback` 으로 직접 미션을 전달하며, feedback(거리·ETA·recovery 횟수)과 result(SUCCEEDED/ABORTED)를 자동 수신한다. 다른 노드와의 통합을 위해 `goal_relay_node.py`를 구현하였다. `/simple_goal`(PoseStamped) 토픽 구독→`navigate_to_pose` action 호출→`/navigation_result`(String) 발행 구조로, 비전 노드 또는 외부 task scheduler가 좌표를 발행하고 결과를 구독하는 형태의 통합이 가능하다. 진행 중 새 goal 수신 시 자동 preemption을 수행한다.

## 6. 결과

`warehouse_world` 전체(30 × 24 m)에 대해 1회 실행으로 자율 매핑이 완료되었으며, 후처리한 맵 기반에서 RViz Goal Pose 또는 `/simple_goal` 토픽 발행을 통해 임의 위치로의 자율 주행이 안정적으로 수행됨을 확인하였다. 좁은 중앙 게이트(2 m 폭)도 통과 가능하며, 도착 yaw까지 정확히 정렬한다. 시뮬레이션에서 검증된 본 파이프라인은 Gazebo 부분 제거 후 실제 라이다·IMU·모터 드라이버 노드로 교체하여 라즈베리 파이-노트북 분산 ROS 2 환경에서 그대로 동작 가능하도록 설계되었다.

## 7. 향후 계획

- `robot_localization` EKF 노드를 통한 wheel odom + IMU 융합으로 충돌 시 odom drift 완화
- 컴퓨터 비전 노드와 `goal_relay_node` 인터페이스 결합하여 객체 위치 기반 자동 미션 수행
- 실물 로봇으로의 이식: ROS_DOMAIN_ID 기반 분산 노드 운영, headless `mission.launch.py` 구조 활용

---

*본 보고서의 디버깅 상세는 `autonomous_exploration_debug_report.md` 참조.*
