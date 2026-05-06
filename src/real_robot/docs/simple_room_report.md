# real_robot — simple_room 시뮬 환경 구축 보고서

작성일: 2026-05-07
대상 패키지: `real_robot` (신규), `ur3_4040_urdf` (변경 없음)
환경: ROS 2 Jazzy / Gazebo Harmonic / `ROS_DOMAIN_ID=28`

---

## 1. 배경

`ur3_4040_urdf` 의 기존 warehouse 시뮬(30 × 24 m)은 조원이 합판으로 실물 환경을 재현하기엔 너무 큼. 실물 로봇(라즈베리파이 5 + S2PRO 라이다 + BNO055 IMU) 운영을 위해 **작은 방 시뮬 환경(simple_room)** 이 별도로 필요.

핵심 요구:
- 합판으로 실물 재현 가능한 사이즈
- 단순 직사각형 + 칸막이 + 장애물 한 개 (탐색 시나리오 의미 있게)
- robot 출발점 명시
- 비전 시스템(천장 CCTV) 좌표계와 일치

---

## 2. 패키지 구조 분리

기존 `ur3_4040_urdf` 는 그대로 두고, 모든 simple_room 관련 자산을 `real_robot` 패키지에 신규 작성.

| 패키지 | 역할 |
|---|---|
| `ur3_4040_urdf` | robot URDF, `nav2_params.yaml`, warehouse 용 launch (변경 없음) |
| `real_robot` | simple_room 의 sdf / launch / maps / slam config |

각 launch 안에서:
- **robot URDF / nav2_params** → `ur3_4040_urdf` 패키지 share 에서 로드
- **world sdf / maps / slam params** → `real_robot` 패키지 share 에서 로드

신규 파일:
```
real_robot/
├── worlds/simple_room.sdf
├── launch/
│   ├── gazebo_simple_room.launch.py
│   ├── map_building_simple_room.launch.py
│   └── navigate_simple_room.launch.py
├── config/slam_params_simple_room.yaml
└── docs/simple_room_report.md  (이 문서)
```

`real_robot/CMakeLists.txt` 의 `install(DIRECTORY ...)` 에 `worlds` 추가, `package.xml` 에 `ur3_4040_urdf`, `ros_gz_sim`, `ros_gz_bridge`, `nav2_bringup`, `explore_lite` 등 13개 exec_depend 추가.

---

## 3. SDF 설계

### 3.1 방 사이즈 결정 흐름

| 시점 | 가로 × 세로 | 결정 사유 |
|---|---|---|
| 초기 제안 | 5.0 × 4.0 m | robot 회전·통과 여유 우선 |
| 1차 축소 | 3.5 × 2.8 m | 합판 작업 분량 부담 |
| 2차 (세로 직사각형) | 2.8 × 4.5 m | "탐색 의미"가 있게 직사각형 |
| **최종** | **3.0 × 5.0 m** | 통로 1.4m + spawn 마진 확보 |

### 3.2 최종 좌표 (방 정중앙 = world (0,0))

- 방 내부: x ∈ [-1.5, +1.5], y ∈ [-2.5, +2.5]
- 벽 두께 0.1m, 높이 0.8m

| 객체 | pose (x, y, z) | size (X × Y × Z) |
|---|---|---|
| 북쪽 벽 | (0, +2.55, 0.4) | 3.2 × 0.1 × 0.8 |
| 남쪽 벽 | (0, -2.55, 0.4) | 3.2 × 0.1 × 0.8 |
| 서쪽 벽 | (-1.55, 0, 0.4) | 0.1 × 5.0 × 0.8 |
| 동쪽 벽 | (+1.55, 0, 0.4) | 0.1 × 5.0 × 0.8 |
| 칸막이 | (-0.2, -0.7, 0.4) | 0.1 × **3.6** × 0.8 |
| 책상 | (-0.9, -1.5, 0.3) | 0.6 × 0.4 × 0.6 |

칸막이는 남쪽 벽(y=-2.5)에 부착, 위쪽으로 3.6m → 끝점 y=+1.1 → **위쪽 통로 폭 1.4m**.

### 3.3 robot spawn

| 항목 | 값 |
|---|---|
| 위치 | (0.8, -1.0, 0.15) |
| 자세 (yaw) | π/2 = 90° (북향) |
| 동쪽 벽까지 | 0.7m |
| 칸막이까지 | 1.0m |
| 남쪽 벽까지 | 1.5m |

---

## 4. SLAM 파라미터 조정 (`slam_params_simple_room.yaml`)

### 4.1 증상
부트스트랩 회전 22초 동안 lidar 빔은 RViz `/scan` display 에 정상 표시되는데 `/map` 토픽으로 통합 안 됨. spawn 자세 + 회전 후 마지막 자세, 두 시점에서만 매핑.

### 4.2 원인
`slam_toolbox` 의 `minimum_score: 0.5` (코드 default) 가 **단조 환경**(대칭에 가까운 simple_room) 에서 scan match 신뢰도를 통과 못 함. simple_room 의 lidar scan 모양이 여러 자세에서 비슷해 매칭 모호 → SLAM 이 그 keyframe 등록 거부 → occupancy grid 갱신 안 됨.

### 4.3 적용한 변경

`real_robot/config/slam_params_simple_room.yaml` 신규 작성. default `mapper_params_online_async.yaml` 복사 후 다음 변경:

| 파라미터 | default | 변경 후 | 이유 |
|---|---|---|---|
| `minimum_score` | 0.5 (코드) | **0.3** | 단조 환경 매칭 통과 |
| `minimum_travel_distance` | 0.5 | 0.1 | 작은 이동도 keyframe |
| `minimum_travel_heading` | 0.5 | 0.1 (≈5.7°) | 작은 회전도 keyframe |
| `minimum_time_interval` | 0.5 | 0.2 | 더 자주 처리 |
| `map_update_interval` | 5.0 | 1.0 | /map 자주 발행 |
| `link_match_minimum_response_fine` | 0.1 | 0.05 | link 매칭 임계 낮춤 |

`map_building_simple_room.launch.py` 의 `slam_launch` 에 `slam_params_file` 인자로 전달.

### 4.4 trade-off

- **단조 환경**: scan match score 가 본래 낮음(0.3~0.5) → 임계치 낮춰야 매핑됨. 잘못된 매칭이 박혀도 환경이 작아 누적 오차 영향 작음.
- **복잡 환경**: scan match score 가 0.7~0.9 로 높음 → default 0.5 그대로 안전. 낮추면 noise 매칭이 박혀 wall 휨/map 일그러짐.

`minimum_score: 0.3` 은 simple_room 같은 좁은 단조 방 전용. warehouse 는 default 0.5 유지.

---

## 5. 부트스트랩 회전 (원형 궤적)

### 5.1 시도 흐름

| 시점 | 명령 | 시간 | 결과 |
|---|---|---|---|
| 초기 (warehouse 동일) | `angular.z=0.5` 제자리 회전 | 14s | simple_room 에서 wheel slip + 좁아 벽 충돌 |
| 1차 | `angular.z=0.3` (slip 감소) | 22s | 충돌 ↓, 그래도 회전 중 매핑 거부 |
| 시도 후 폐기 | 22s 회전 → 3s 정지 → 22s 회전 | 47s | 효과 미미, 단순 회전 본질 동일 |
| **최종** | **`linear.x=0.05, angular.z=0.3`** (반경 0.17m 원형) | 22s | 직진 성분이 SLAM keyframe trigger → 매핑 갱신 |

### 5.2 원형 궤적의 의의

SLAM Toolbox 가 **rotation-only** 일 때 keyframe 등록을 거부하는 정책 의심됨. 매우 작은 직진 (`linear.x=0.05 m/s`) 추가로 `minimum_travel_distance` trigger → keyframe 등록 → 회전 중에도 매핑 갱신.

반경 0.17m 라 22초 동안 robot 이 spawn 위치(±0.17m) 안에서 머무름 → 좁은 simple_room 에서도 안전.

---

## 6. Nav2 파라미터 (보수적 주행)

### 6.1 증상
robot 이 벽에 너무 가깝게 path 따라감. 회전 시 chassis 모서리 sweep 으로 벽 충돌 위험.

### 6.2 적용한 변경 (RewrittenYaml override, 매핑 단계 한정)

매핑 단계만 영향 받도록 `map_building_simple_room.launch.py` 에서 `RewrittenYaml` 로 override. `ur3_4040_urdf/config/nav2_params.yaml` 자체는 미변경.

| 파라미터 | default | 변경 후 |
|---|---|---|
| `yaw_goal_tolerance` | 0.25 | 3.14 (매핑 동안 frontier blacklist 방지) |
| `inflation_radius` | 0.25 | **0.4** |
| `cost_scaling_dist` | 0.6 | 1.0 |
| `cost_scaling_gain` | 1.5 | 2.5 |
| `lookahead_dist` | 0.4 | 0.3 |
| `max_lookahead_dist` | 0.6 | 0.45 |
| `max_allowed_time_to_collision_up_to_carrot` | 1.0 | 1.5 |
| `desired_linear_vel` | 0.35 | 0.25 |

이 조합으로 path 가 **벽에서 robot 외접원(0.337m) + inflation(0.4m) ≈ 0.74m** 떨어진 곳에 만들어짐.

### 6.3 시도 후 폐기한 추가 보수화

`inflation_radius: 0.55` + `footprint_padding: 0.1` 동시 적용 시도.

**결과**: 통로 폭 1.4m 에서 양쪽 inflation 1.1m 가 점유 → robot 중심선 path 가능 영역이 0.3m 만 남음 → footprint_padding 까지 더해져 cost 가 매우 높아짐 → planner 가 통로 path 를 비현실 판단으로 거부 → 칸막이 너머 frontier blacklist → 매핑이 절반에서 종료.

**교훈**: 좁은 환경에서 inflation 과 footprint_padding 동시 강화하면 통로 통과 자체를 막음. **`inflation_radius: 0.4` 가 simple_room 1.4m 통로의 sweet spot**. 더 보수화하려면 통로 폭을 늘리거나, 속도 줄이는 방향으로 가야 함.

---

## 7. 좌표계 통일 (world frame 정적 TF)

### 7.1 배경
사용자 운영 계획: 천장 CCTV 로 비전 시스템 구축 → 비전이 인식한 물체 좌표를 nav2 goal 로 그대로 전달하려면 비전 ↔ robot 의 **좌표계 일치 필수**.

### 7.2 문제: 좌표계 셋이 어긋남

| 좌표계 | (0, 0) 위치 |
|---|---|
| Gazebo world | 방 정중앙 (SDF 기준) |
| SLAM map | robot spawn 위치 (Gazebo world 의 0.8, -1.0) |
| CCTV 캘리브레이션 (예정) | 너가 정하기 나름 |

### 7.3 시도와 실패

slam_toolbox 의 `map_start_pose: [0.8, -1.0, 1.5708]` 옵션 시도. 의도: SLAM 이 시작 시점 robot 위치를 명시한 좌표로 인식 → map (0,0) 이 방 정중앙(=world (0,0)) 과 일치.

**실패**: `tf2_echo map base_footprint` 결과 spawn 위치(0.8, -1.0) 가 안 나오고 robot 의 부트스트랩 후 위치(0.179, 0.168) 가 나옴. `map_start_pose` 가 새 매핑 시작 시 무시됨(기존 map 이어 매핑 시에만 적용되는 옵션).

### 7.4 해결: world → map 정적 TF

`map_building_simple_room.launch.py` 에 `world` 라는 새 frame 추가. `static_transform_publisher` 로 `world → map` 변환 발행:

```python
arguments=['0.8', '-1.0', '0', '1.5708', '0', '0', 'world', 'map']
# x   y     z   yaw      pitch roll  parent  child
```

해석: map frame 의 origin 은 world 기준 (0.8, -1.0) 위치, yaw=π/2 회전.

### 7.5 운영 시 활용

- **world frame** = 방 정중앙 = CCTV 캘리브레이션 좌표
- **map frame** = SLAM 의 robot spawn 기준 (그대로)
- 비전 노드가 발행하는 좌표를 `frame_id: 'world'` 로 publish → nav2 가 받아 자동으로 map frame 으로 변환 후 navigation
- 시뮬과 실물 모두 동일 패턴 — 실물 매핑 시 robot 을 정확히 같은 위치/자세에 두고 SLAM 시작하면 모든 좌표계가 일관됨

---

## 8. 최종 검증 결과

- **매핑**: simple_room 전체(외벽 4면 + 칸막이 + 책상) 매핑 가능
- **robot 주행**: 벽에서 ~0.74m path 확보, 통로(1.4m) 통과 가능
- **좌표계**: world ↔ map 자동 변환, 비전 시스템 통합 준비 완료
- **부트스트랩**: 22초 원형 궤적으로 robot 주변 360° 매핑 갱신

### 사용 명령

```bash
# 빌드
cd ~/ur3
colcon build --symlink-install --packages-select real_robot
source install/setup.bash

# 매핑
ros2 launch real_robot map_building_simple_room.launch.py

# 매핑 저장
cd ~/ur3/src/real_robot/maps
ros2 run nav2_map_server map_saver_cli -f simple_room_map \
  --ros-args -p map_subscribe_transient_local:=true

# Navigation
ros2 launch real_robot navigate_simple_room.launch.py
```

---

## 9. 제한사항 / 향후 과제

### 9.1 매핑 한계
- 단조 환경이라 `minimum_score: 0.3` 으로도 일부 회전 구간 매핑이 부분적
- robot 자신이 가린 영역(footprint + lidar `min_range: 0.35m` 안쪽)은 부트스트랩만으론 unknown 잔존 → 매핑 후 추가 navigate 로 보완 필요

### 9.2 nav2 보수화 한계
- 통로 폭 1.4m + inflation 0.4m = path 가능 영역 0.6m. 더 보수화 시 통로 통과 불가
- 회전 시 chassis sweep 의 절대적 보호는 inflation/footprint 만으론 어려움 — 속도 감소 + lookahead 단축으로 간접 완화

### 9.3 실물 이전 시 변경점

| 시뮬 (현재) | 실물 |
|---|---|
| `gazebo_simple_room.launch.py` (gz spawn + bridge) | 제거 |
| gz-sim diff_drive plugin (`/odom`, `/cmd_vel`) | 모터 드라이버 노드 |
| gpu_lidar 시뮬 (`/scan`) | rplidar driver 등 |
| gz-sim IMU | bno055 driver |
| `world → map` 정적 TF | 동일 패턴 그대로 사용 (실물에서도 robot 시작 위치 명시) |
| `slam_params_simple_room.yaml` | 그대로 사용 |
| `nav2_params.yaml` + RewrittenYaml override | 그대로 사용 |

`map_building_simple_room.launch.py` 의 gazebo include 부분만 실물 driver launch 로 교체하면 됨.

### 9.4 향후 개선 후보
- 비전 노드 통합 (현재는 좌표계만 준비, 노드 자체는 미작성)
- AMCL 의 `set_initial_pose` 를 spawn 좌표(0.8, -1.0, 1.5708) 로 명시 → `navigate_simple_room.launch.py` 에서 자동 초기화
- 매핑 결과 후처리 (`scripts/clean_map.py` 응용해 simple_room 의 lidar self-hit 잔여물 정리)

---

## 부록 A: 참고

- 기존 ur3_4040_urdf 디버깅 보고서: `src/ur3_4040_urdf/docs/autonomous_exploration_debug_report.md` (warehouse 환경 매핑/주행 진단 1~12절)
- read_me.txt: `src/ur3_4040_urdf/read_me.txt` (워크스페이스 setup + warehouse 사용법)

---

작성: 2026-05-07
