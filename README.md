# capstone — 실물 로봇 매핑 & 내비게이션 워크스페이스

차동구동 로봇(이동베이스 + UR3 팔, "L_AND_F")의 SLAM 매핑과 Nav2 자율주행을 위한
통합 ROS 2 (Jazzy) colcon 워크스페이스.

## 1. 개요

`~/ur3`, `~/ros2_pid_control`, `~/lidar_ws` 등에 흩어져 있던 매핑/내비게이션 코드를
하나로 통합한 워크스페이스. 시뮬레이션(Gazebo)에서 실물 로봇으로 이행하는 캡스톤 프로젝트.

**분산 구조 — 컴퓨터 2대:**

```
       [라즈베리파이]                          [PC / 노트북]
  라이다 ─ ydlidar_driver ──/scan────┐    ┌── slam_toolbox / Nav2
  IMU    ─ bno055_driver  ──/imu/data─┼──→ ┤   robot_state_publisher
                                      │    └── serial_bridge ──USB──→ ESP32 ──→ 모터
                            (ROS 2 DDS · 동일 ROS_DOMAIN_ID)
```

- **PC**: SLAM, Nav2, 모터 제어(serial_bridge), RViz
- **라즈베리파이**: 센서 드라이버(라이다·IMU)만 실행해 토픽을 PC로 전달
- ESP32 모터컨트롤러는 PC에 USB 시리얼로 연결
- 두 머신은 같은 네트워크 + 같은 `ROS_DOMAIN_ID`

## 2. 하드웨어

| 구성 | 내용 |
|---|---|
| 로봇 | 차동구동 이동베이스 + UR3 팔 ("L_AND_F") |
| 모터 제어 | ESP32 (PID 펌웨어) — 펌웨어는 `~/ros2_pid_control`에 별도 보관 |
| 라이다 | YDLidar X4 Pro (내부 펌웨어는 S2 Pro로 표기) — 라즈베리파이 USB |
| IMU | BNO055 (I2C, 주소 0x29) — 라즈베리파이 I2C 버스 1 |
| 센서 컴퓨터 | Raspberry Pi (ROS 2 Jazzy) |

## 3. 패키지 구조

```
capstone/src/
├── robot_bringup/        런치파일 + SLAM설정 + 맵 (워크스페이스 진입점)
│   ├── launch/  mapping.launch.py · navigation.launch.py · sensors.launch.py
│   ├── config/  slam_real.yaml
│   └── maps/    my_real_map.{pgm,yaml}
├── ros2_pid_bridge/      ESP32 ↔ ROS 2 브리지 (/cmd_vel→모터, 엔코더→/odom+TF)   [PC]
├── ur3_4040_urdf/        로봇 URDF/xacro + nav2_params.yaml + ekf.yaml           [PC]
├── bno055_driver/        BNO055 IMU 드라이버 (Python / adafruit_bno055)          [RPi]
└── ydlidar_ros2_driver/  YDLidar 드라이버                                        [RPi]
```

워크스페이스는 PC·라즈베리파이 양쪽에 동일하게 존재하며, 각 머신은 자기가 쓸 패키지만 실행한다.

## 4. 빌드

PC·라즈베리파이 공통:

```bash
cd ~/capstone
colcon build --symlink-install
source install/setup.bash
```

라즈베리파이는 `ws_setting capstone` 로 ROS 2 + 워크스페이스를 한 번에 활성화할 수 있다.

## 5. 실행

모든 터미널에서 먼저 `source ~/capstone/install/setup.bash`, PC·라즈베리파이 `ROS_DOMAIN_ID` 동일이 전제다.
실기 운용에서 외울 명령은 아래 launch 4개 + 맵 저장 1개뿐이다.

| 단계 | 라즈베리파이 | PC |
|---|---|---|
| 매핑 | `sensors.launch.py` | `mapping.launch.py` + 텔레옵 + `map_saver_cli` |
| 내비게이션 | `sensors.launch.py` | `navigation.launch.py` |

### 5-1. 센서 (라즈베리파이) — 매핑·내비게이션 공통 1단계

```bash
ros2 launch robot_bringup sensors.launch.py                # 라이다 + IMU
ros2 launch robot_bringup sensors.launch.py lidar:=false   # IMU만 (라이다 없을 때)
```

**무슨 일이 일어나나:** YDLidar(라이프사이클 노드 — 자동으로 configure→activate)와
BNO055 IMU 드라이버가 뜬다. 결과로 `/scan`, `/imu/data`, `/imu/mag` 토픽이 DDS로
발행되어 PC가 받아 쓴다. (`imu:=false` 로 IMU만 제외 가능)

### 5-2. 매핑 (PC)

```bash
ros2 launch robot_bringup mapping.launch.py
ros2 launch robot_bringup mapping.launch.py port:=/dev/ttyUSB1   # ESP32 포트가 바뀐 경우
# 별도 터미널 — 텔레옵으로 로봇을 천천히 운전하며 공간을 한 바퀴
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 맵 저장
ros2 run nav2_map_server map_saver_cli -f ~/capstone/src/robot_bringup/maps/my_real_map
```

**`mapping.launch.py` 가 PC에서 띄우는 것:**

| 노드 | 하는 일 |
|---|---|
| `robot_state_publisher` | URDF로 TF 트리 발행 (`base_footprint`→`base_link`→`lidar_link`/`imu_link` …) |
| `serial_bridge` | ESP32와 시리얼 통신. `/cmd_vel`을 받아 모터로 보내고, 엔코더로 `/odom` + `odom`→`base_footprint` TF 발행 |
| `slam_toolbox` | `/scan` + `/odom`을 합쳐 지도를 그리고 `map`→`odom` TF 발행 (`slam_real.yaml` 사용) |
| `rviz2` | 지도가 그려지는 과정을 실시간 확인 |

**텔레옵**은 키보드 입력을 `/cmd_vel`로 발행 → `serial_bridge`가 ESP32로 전달 → 로봇이 움직인다.
방을 한 바퀴 돌면 RViz에서 지도가 채워지고, **맵 저장** 명령이 현재 지도를
`my_real_map.pgm`(이미지) + `my_real_map.yaml`(메타)로 떨어뜨린다 — 다음 단계의 입력 맵.

### 5-3. 내비게이션 (PC)

```bash
ros2 launch robot_bringup navigation.launch.py
ros2 launch robot_bringup navigation.launch.py map:=/경로/다른맵.yaml   # 다른 맵 사용
ros2 launch robot_bringup navigation.launch.py port:=/dev/ttyUSB1      # ESP32 포트가 바뀐 경우
```

**`navigation.launch.py` 가 PC에서 띄우는 것** (SLAM 대신 "저장된 맵 + AMCL 위치추정"이 들어간 게 매핑과의 차이):

| 시점 | 노드 | 하는 일 |
|---|---|---|
| 즉시 | `robot_state_publisher` + `serial_bridge` | 매핑 때와 동일 (TF, 모터, `/odom`) |
| +3초 | Nav2 **Localization** | `map_server`가 저장된 맵을 불러오고 `AMCL`이 라이다로 위치 추정 (`map`→`odom` TF) |
| +10초 | Nav2 **Navigation** | `planner`(경로 계획) / `controller`(주행 제어) / `behavior` / `bt_navigator` (`nav2_params.yaml` 사용) |
| 즉시 | `rviz2` | 목표 지정 및 경로 확인 |

띄운 뒤 RViz에서: **"2D Pose Estimate"** 로 지도 위 로봇의 실제 위치·방향을 클릭·드래그해
AMCL을 초기화하고 → **"Nav2 Goal"** 로 목표 지점을 찍으면 로봇이 스스로 경로를 짜 자율주행한다.

## 6. TF 트리

```
map ──(slam_toolbox / AMCL)──▶ odom ──(serial_bridge)──▶ base_footprint
                                              └─(robot_state_publisher)─▶ base_link ─▶ lidar_link / imu_link / 바퀴 …
```

## 7. git 동기화 (PC ↔ 라즈베리파이)

PC가 작업 머신, 라즈베리파이는 git으로 변경을 받는다.

**PC에서 (변경 후):**
```bash
cd ~/capstone
git add -A
git commit -m "변경 내용"
git push rpi
```

**라즈베리파이에서 (변경 받기):**
```bash
cd ~/capstone
git pull
colcon build --symlink-install
```

`build/`, `install/`, `log/` 는 git 추적에서 제외된다(`.gitignore`) — 머신마다 따로 빌드.

## 8. 현재 상태 & 로드맵

> 상세 진행 현황·결정 배경·주말 작업 체크리스트: **[docs/STATUS.md](docs/STATUS.md)**

**완료**
- 워크스페이스 통합, PC·라즈베리파이 양쪽 빌드
- BNO055 IMU — 라즈베리파이에서 작동 확인 (`/imu/data`, IMUPLUS 모드)

**대기 (하드웨어)**
- 라이다, 실물 로봇 — 도착 후 진행

**브링업 로드맵**
1. 텔레옵으로 첫 맵 작성 + odom/TF 검증
2. PID 튜닝 (ESP32 펌웨어의 `GAIN:` 시리얼 명령)
3. robot_localization EKF (IMU + 휠 오도메트리 융합) — `ekf.yaml` 수정 필요
4. explore_lite 자율 탐사

## 9. 주의사항

- **ESP32 펌웨어는 이 워크스페이스에 없다** — `~/ros2_pid_control` (PlatformIO 프로젝트)
- `WHEEL_BASE = 0.542 m` 는 URDF 기준값 — 실측 후 펌웨어/런치파일 보정 필요
- ESP32 시리얼 포트(`/dev/ttyUSB*`)는 USB 재연결 시 번호가 바뀔 수 있다 → 런치 인자 `port:=` 로 지정
- PC ↔ 라즈베리파이 `ROS_DOMAIN_ID` 는 반드시 동일해야 토픽이 보인다
