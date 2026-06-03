# 캡스톤 로봇 프로젝트 — 진행 현황 & 개발 노트

> 이 문서는 프로젝트의 전체 맥락·결정사항·현황·다음 할 일을 한곳에 모은 것이다.
> 빌드·실행 명령(사용법)은 워크스페이스 루트의 [README.md](../README.md) 참고.
> 최종 갱신: 2026-06-02 (오후)

---

## 1. 프로젝트 개요

ESP32 기반 차동구동 로봇(이동베이스 + UR3 팔, 로봇명 "L_AND_F")을 **시뮬레이션에서 실물로** 옮겨
SLAM 매핑 + Nav2 자율주행을 구현하는 캡스톤 프로젝트.

`~/ur3`, `~/ros2_pid_control`, `~/lidar_ws` 등에 흩어져 있던 코드를 `~/capstone` 워크스페이스 하나로 통합했다.

**분산 구조** — 컴퓨터 2대:
- **PC(노트북)**: SLAM, Nav2, 모터 제어(serial_bridge), RViz
- **라즈베리파이**: 센서 드라이버(라이다·IMU)만 실행해 토픽을 PC로 전달
- 같은 네트워크 + 같은 `ROS_DOMAIN_ID`

---

## 2. 하드웨어 현황

| 구성 | 상태 |
|---|---|
| 로봇 (이동베이스 + UR3 팔) | ✅ 있음 (학교 보관) |
| ESP32 모터컨트롤러 | ✅ 있음, 모터 구동 가능 |
| 라이다 YDLidar X4 Pro (펌웨어는 S2 Pro로 표기) | ✅ 있음 |
| IMU BNO055 | ✅ 있음, 작동 검증 완료 |
| 라즈베리파이 | ✅ 있음, ROS 2 Jazzy |
| **엔코더** | ✅ **복구 완료** (2026-05-27) |

### 엔코더 — 해결됨 (이전: 5V↔3.3V 레벨컨버터 한 채널 고장)

레벨컨버터 교체 완료, ESP32에서 좌·우 엔코더 모두 정상 값 수신 확인.

**이제 가능해진 것:**
- 휠 오도메트리 (`/odom`) — `serial_bridge`로 정상 발행
- ESP32 PID 폐루프 주행 — 엔코더 피드백으로 PID 게인 튜닝 가능
- robot_localization EKF (엔코더 + IMU 융합) 진행 가능
- Nav2 자율주행 경로 열림

---

## 3. 완료된 작업

- **워크스페이스 통합** — `~/capstone`, 5개 패키지 (4절 참고)
- **git 저장소화 + PC↔RPi 동기화** — rsync 대신 git push/pull
- **IMU(BNO055) 검증** — 라파에서 `/imu/data` 발행 확인, IMUPLUS 모드 적용
- **`ekf.yaml` crash 수정** — 파라미터 배열의 정수/실수 혼용 버그 (`ekf_node` 정상 기동 확인)
- **README.md 작성** — 워크스페이스 사용법 문서
- ESP32 펌웨어: 런타임 PID 게인 조정(`GAIN:` 시리얼 명령) 추가, `platformio.ini` 빌드 오타 수정

### 5월 28일 ~ 6월 2일 (실물 브링업)

- **엔코더 복구** — 레벨컨버터 교체 후 양쪽 엔코더 정상 수신
- **캘리브레이션 완료** (실측 + 회전 테스트):
  - `WHEEL_RADIUS = 0.0625` m (지름 12.5cm 실측)
  - `TICKS_PER_REV = 11400` (19 PPR × 4 quadrature × 150 gear, IG-52GM)
  - `WHEEL_BASE = 0.58` m (실측 60cm + 휠 슬립 보정)
- **udev 규칙** — `/dev/esp32` 심볼릭 자동 (`/etc/udev/rules.d/99-esp32.rules`),
  USB 꽂는 순서 무관. mapping/navigation launch 기본값도 `/dev/esp32`.
- **PID 게인 런타임 튜닝 토픽** — `/pid_gain` (Float32MultiArray) 추가 (serial_bridge)
- **`/pid_gain` → `GAIN:` 시리얼 변환 + GAIN_SET 응답 로그**
- **EKF 통합 완료** (mapping + navigation 양쪽):
  - `ekf_filter_node` 추가, serial_bridge `publish_tf=false`로 TF 충돌 방지
  - `ekf.yaml` 에서 IMU는 vyaw(yaw rate)만 사용 (절대 yaw OFF — 떨림 fix, 50kg 가속도 캘 불가 [[imu-gyro-only]])
  - 출력: `/odometry/filtered` 토픽 + `odom→base_footprint` TF
- **SLAM Toolbox 파라미터 튜닝** — minimum_travel_distance/heading ↑, correlation_search_space ↓, loop_match 엄격
- **automatic 회전 테스트 스크립트** — `scripts/rotation_test.py` (/odom 보고 360° 회전 후 정지)
- **navigation.launch.py 완전 재작성** — mapping.launch.py 와 같은 자동화 구조 (RPi SSH, EKF, 헬스체크 포함)
- **BRINGUP_CHECKLIST.md** — 실물 첫 매핑 절차 문서

### 알아낸 본질적 제약

- **휠 슬립**: 50kg 로봇 + 불균일 바닥 → 슬립 일정하지 않음. EKF + wheelbase 캘로 ~50% 잡았지만 잔여 누적 오차 불가피.
- **광범위 매핑 시 맵 구조 왜곡**: 슬립 누적으로 벽이 실제 위치 ≠ 맵 위치 → Nav2 가 환영 벽으로 인식해서 이상한 경로 계획.
- **결론 (전략 변경)**: 넓은 공간 욕심 X, **작은 공간(2-3m) 정밀 매핑 + Nav2 안정 동작** 으로 데모. ([[demo-strategy-small-area]])

### 6월 2일 저녁 (집에서 — 코스트맵 사전 검증)

- **`costmap_preview.launch.py` 추가** ([[offline-preview-pattern]]):
  - 실물 로봇 없이 저장된 맵 + nav2_params 로 글로벌 코스트맵만 띄움.
  - 학교/집 분리 워크플로우 — 집에서 inflation·footprint 검증.
  - 한계: Local costmap / 라이다 실시간 부분은 미리보기 불가.
- **코스트맵 시각적 확인**: 0.35 inflation 적용된 상태에서 벽 주변 그라데이션 두께 충분히 부풀어 있음 ✅ (사용자 평가 "괜찮은 것 같다").
  - 만약 학교 가서 여전히 벽 부딪히면 → AMCL 위치 오차 또는 컨트롤러 추종 실패 의심 (실물 환경 필요).

### 6월 2일 오후 (캘리브레이션 + EKF 정제 + Nav2 시작)

- **엔코더 비대칭 문제 해결** ([[ekf-sensor-split-and-upload-safety]]):
  - 직진만 했는데 /odom 이 41° 옆으로 휘는 현상 → 좌우 엔코더 미세 비대칭
  - EKF 설정 변경: `/odom` 은 `vx` 만, `/imu/data` 는 `vyaw` 만. 가짜 회전 완전히 제거됨.
- **거리 캘 재발견**: TICKS_PER_REV 11400 → **5700** 으로 절반. 1m 직진 테스트에서 97.8% 정확 (0.978m).
  - 엔코더 한 채널 신호가 약해 quadrature ×2 decoding 만 동작하는 듯.
- **자동 펌웨어 업로드** (`skip_upload` 기본 `false`): 사용자가 "낭비여도 확실한 게 좋다" 고 요청.
  - launch 전 모터 전원 OFF 절차 강조 (부트로더 단계 GPIO 떠 모터 구동 위험).
- **SLAM 파라미터 추가 완화**:
  - `loop_match_minimum_response_fine`: 0.45 → **0.35** (loop closure 더 자주 트리거)
  - `scan_buffer_size`: 35, `loop_search_maximum_distance`: 5.0 유지
- **Nav2 inflation_radius**: `0.25` → **`0.35`** (벽에서 더 떨어져 다님, 사용자 요청)
- **맵 GIMP 수동 보정 진행 중** — 잘못된 벽 지우기, 노이즈 정리.
- **navigation.launch.py 완전 재작성**: mapping.launch.py 와 동일 구조 (RPi SSH, EKF, 자동 업로드, 헬스체크).

---

## 4. 워크스페이스 구조 (5개 패키지)

```
capstone/src/
├── robot_bringup/        런치파일 + SLAM설정 + 맵 (워크스페이스 진입점)
├── ros2_pid_bridge/      ESP32 ↔ ROS 2 브리지                  [PC]
├── ur3_4040_urdf/        로봇 URDF + nav2_params + ekf.yaml     [PC]
├── bno055_driver/        BNO055 IMU 드라이버 (Python)           [RPi]
└── ydlidar_ros2_driver/  YDLidar 드라이버                       [RPi]
```

자세한 트리·역할은 [README.md](../README.md) 3절 참고.

---

## 5. 주요 결정과 이유

- **분산 구조** — 라파는 센서 토픽만 PC로 넘기고, 무거운 연산(SLAM/Nav2)은 PC에서.
- **IMU = IMUPLUS 모드** — 지자기(magnetometer) 미사용. 금속 차체 + 모터가 자기장을 오염시켜
  지자기를 못 믿기 때문. 자이로 캘리브레이션은 "로봇 가만히 두기"만으로 자동 → 로봇을 뒤집을 필요 없음.
- **EKF는 로드맵 3단계** (지금 런치에 미포함) — EKF는 휠 오도메트리 + IMU를 융합하는 것.
  휠 오도메트리만으로도 SLAM은 되고, 지금은 엔코더 고장 상태라 라이다 SLAM을 먼저 한다.
- **explore_lite(자율 탐사)는 4단계** — PID·Nav2 검증 후.
- **`real_robot` 패키지 제거** — 시뮬 잔재라, SLAM 설정·맵만 `robot_bringup`에 흡수.
- **git은 GitHub 미사용** — PC↔RPi 직접 연결. 커밋 identity: `innhyung` / `nblue12@gmail.com`.

---

## 6. 핵심 개념

- **TF 트리:** `map → odom → base_footprint → base_link → (lidar_link / imu_link / 바퀴)`
- **EKF 역할:** 휠 오도메트리(위치 정확)와 IMU(방향 정확)를 융합. IMU를 측위에 쓰는 통로.
- **엔코더 없이 매핑이 되는 이유:** `slam_toolbox`가 연속 라이다 스캔을 정합(scan matching)해서
  로봇의 움직임을 직접 계산한다. 라이다 = 위치, IMU = 방향. 엔코더는 안 써도 된다.
- **IMU만으로는 불가:** IMU는 위치를 모른다(가속도 이중적분 → 수 초 만에 드리프트). 라이다가 있어야 한다.

---

## 7. 브링업 로드맵

1. ~~라이다 SLAM으로 첫 매핑~~ ✅ 완료 (2026-05-28)
2. ~~레벨컨버터 교체 → 엔코더 복구~~ ✅ 완료 (2026-05-27)
3. ~~PID 게인 런타임 튜닝~~ ✅ 완료 (`/pid_gain` 토픽)
4. ~~robot_localization EKF 통합~~ ✅ 완료 (2026-06-02, mapping + navigation 양쪽)
5. ~~거리 캘리브레이션 (TICKS_PER_REV 5700)~~ ✅ 완료 (2026-06-02, 1m 테스트 97.8%)
6. ~~엔코더 비대칭 → EKF 분담 조정~~ ✅ 완료 (2026-06-02)
7. **(진행 중)** GIMP 맵 보정 + Nav2 첫 시연
8. **(다음)** Nav2 미세 튜닝 — `inflation_radius` 추가 조정, `max_vel` 등 (지금 0.35 적용됨, 내일 효과 확인)
9. **(다음)** 데모 시나리오 확정 + 안정성 반복 검증
10. 영상 녹화 (6월 중순)
11. 보고서 / 발표 자료 (6월 15일~17일)
12. explore_lite 자율 탐사 (스트레치 — 시간 남으면)

---

## 8. 주말(학교) 작업 체크리스트

순서대로 진행:

### A. 라이다 확인 (라즈베리파이)
```bash
# 라이다 USB를 라파에 연결한 뒤
ls /dev/ttyUSB*                                          # 포트 확인
ws_setting capstone                                      # ROS2 + 워크스페이스 활성화
ros2 launch robot_bringup sensors.launch.py imu:=false   # 라이다만 먼저
# 새 터미널에서
ros2 topic echo /scan --once                             # 스캔 데이터 확인
```
`/scan`이 나오면 성공. 안 나오면 `ydlidar_ros2_driver/params/ydlidar.yaml`의
`port`·`baudrate`를 점검.

### B. 라이다 드라이버 버전 확정
- capstone의 ydlidar 드라이버로 잘 되면 → 라파의 `~/robot_ws`를 삭제해도 됨
- 안 되면 → `~/robot_ws/src/ydlidar_ros2_driver`를 capstone으로 복사 후 재빌드
  (두 버전 차이는 사실상 주석·코드스타일뿐이라 capstone 것으로 될 가능성이 높음)

### C. 센서 통합 검증
```bash
ros2 launch robot_bringup sensors.launch.py    # 라이다 + IMU 동시
```
PC에서 `ros2 topic list`로 `/scan`, `/imu/data`가 보이는지 확인 (ROS_DOMAIN_ID 동일 필수).

### D. 첫 매핑 (손-매핑)
- ⚠️ **손-매핑용 scan-only SLAM 런치는 아직 미작성** — 이 단계에 갈 때 만들어야 함
  (다음 대화에서 "scan-only 매핑 런치 만들어줘"라고 요청).
- 개념: 엔코더·모터 없이 `slam_toolbox` 스캔정합 + 가짜 `odom` TF.
  로봇에 라이다·IMU·RPi를 얹고 전원 → SLAM 실행 → 로봇을 손으로 밀며 공간을 한 바퀴 →
  RViz에서 지도 확인 → `map_saver_cli`로 저장.

---

## 9. 미결 / 주의사항

- `WHEEL_BASE = 0.542 m`는 URDF 기준값 — 실측 후 펌웨어·런치파일 보정 필요
- **ESP32 펌웨어는 이 워크스페이스에 없음** — `~/ros2_pid_control` (PlatformIO 프로젝트)
- 펌웨어의 `RAW` 디버그 명령은 레벨컨버터 검증용 — 교체·검증 전까지 남겨둘 것
- 라파의 `~/robot_ws`는 `COLCON_IGNORE` 상태로 보존 중 — 라이다 드라이버 확정 후 삭제
- `ROS_DOMAIN_ID`는 PC·라즈베리파이가 반드시 동일해야 토픽이 보인다
- 손-매핑용 SLAM 런치 미작성 (8-D 참고)
- robot_localization EKF 런치 통합 미완 (로드맵 4단계)

---

## 10. 작업 환경 메모

- **PC**: `~/capstone` git 작업 저장소. ESP32를 USB로 연결.
- **라즈베리파이**: `danial@10.42.0.78` (유선) / `172.30.1.83` (WiFi). `~/capstone`은 git 작업본.
  라이다·IMU를 연결. `ws_setting capstone`으로 활성화.
- **git 동기화**: PC에서 `git add -A && git commit && git push rpi`,
  라파에서 `cd ~/capstone && git pull && colcon build` (README 7절 참고).
