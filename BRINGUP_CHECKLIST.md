# 학교 작업 체크리스트 — 실물 브링업 (Day 1+)

> 작성: 2026-05-27 · 데드라인: 2026-06-18
>
> 내일(5/28) 학교 가서 차례대로 할 일. 위에서 아래로 진행하면 됩니다.
> 각 단계의 **목적 / 명령 / 무슨 일이 일어나는가 / 통과 기준**을 같이 적었습니다.
> 막히면 §13 트러블슈팅 참고. 실패해도 다음 단계 영향 없는 건 ⏭️로 표기.

---
---
---

## 2. 하드웨어 연결 (모터 전원은 절대 마지막)

**순서가 중요합니다 — 안전을 위해.**

- [ ] **모터 전원 차단 상태** 확인 (드라이버 메인 전원 OFF, 또는 모터 측 배선 분리)
- [ ] ESP32 USB → **PC**에 연결
- [ ] 라이다 USB → **라즈베리파이**에 연결
- [ ] IMU(I2C) → **라즈베리파이** GPIO 핀 연결 확인 (보통 SDA·SCL + 3.3V + GND)
- [ ] 라파 부팅, 네트워크 연결 확인

**검증:**
```bash
# PC에서
ls /dev/ttyUSB*        # ESP32가 /dev/ttyUSB0 또는 ttyUSB1로 잡혀야 함
# 라파에서 (SSH 후)
ls /dev/ttyUSB*        # 라이다가 잡혀야 함
i2cdetect -y 1         # IMU가 0x29 또는 0x28로 보여야 함
```

`/dev/ttyUSB*` 가 없으면 USB 케이블 / 드라이버 / 권한(dialout 그룹) 점검 — §13-A 참고.

---

---

## 5. PC 측 매핑 launch (모터 전원은 여전히 OFF)

**목적:** ESP32 시리얼·SLAM·TF가 다 살아있는지 한 번에 검증.

```bash
ros2 launch robot_bringup mapping.launch.py
```

**무슨 일 (자동으로):**
1. ESP32 펌웨어 업로드는 **건너뜀** (어제 `skip_upload:=true` 기본 적용)
2. SSH로 라파의 `sensors.launch.py`도 다시 띄움 — §4를 이미 띄웠다면 라파에서 충돌 가능
   - 충돌 피하려면 라파의 sensors는 끄고 launch만 돌리거나
   - 또는 `skip_rpi:=true`로 PC만 띄우고 라파는 §4 그대로 유지

**모터 전원 OFF 상태에서 권장 호출:**
```bash
ros2 launch robot_bringup mapping.launch.py skip_rpi:=true
# (라파 sensors 는 §4에서 이미 돌고 있으므로)
```

**15초 뒤 헬스체크 박스가 같은 터미널에 뜸:**
```
═══════════════ 매핑 시스템 헬스체크 ═══════════════
  ✅ /scan        정상
  ✅ /imu/data    정상
  ✅ /odom        정상   ← 이게 ✅ 이면 ESP32 펌웨어가 살아있다는 뜻
───────────────────────────────────────────────────
```

**`/odom` ✅의 의미:**
- ESP32가 `/dev/ttyUSB0`로 잡혔고
- serial_bridge가 시리얼 열어서 ESP32 리셋 펄스 끝났고
- 펌웨어가 `FB:` 패킷을 보내는 중이고 (PID는 target=0, PWM=0 페일세이프)

**3개 다 ✅면 모터 전원 ON 안전. 이 시점부터 모터 켜도 안 움직임.**

---

## 6. 모터 전원 ON

- [ ] 드라이버 메인 전원 ON
- [ ] LED 들어오는지 확인
- [ ] **모터가 안 움직여야 정상.** 움직이면 즉시 OFF 하고 §13-D.

---

## 7. /odom 정확도 1차 검증 (1m 직진 테스트)

**목적:** 휠 엔코더가 실거리를 정확히 잡는지 확인. 안 맞으면 매핑이 어긋남.

**준비:**
- 평평한 빈 공간 ~1.5m 확보
- 로봇 출발점에 마스킹테이프로 표시
- 별도 터미널 1: `rqt_plot`
- 별도 터미널 2: `ros2 topic echo /odom`

**rqt_plot:**
```bash
ros2 run rqt_plot rqt_plot \
  /cmd_vel/linear/x \
  /odom/twist/twist/linear/x
```

**텔레옵:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.10 -p turn:=0.3
```

**테스트 절차:**
1. 텔레옵 창에서 `i` 키를 약 1초 정도 눌렀다 뗌 (천천히)
2. 로봇이 정확히 멈출 때까지 기다림
3. 줄자로 **실제 이동 거리** 측정 (예: 0.87 m)
4. `ros2 topic echo /odom --once` 에서 `position.x` 값 확인 (예: 0.95)

**판정:**

| 결과 | 의미 | 조치 |
|---|---|---|
| 실제 ≈ /odom 의 x (±5%) | 엔코더 보정 정상 | §8로 진행 |
| /odom 이 실제보다 큼 | 엔코더 카운트 과대 → `ticks_per_rev` 또는 `wheel_radius` 잘못 | §8에서 보정 |
| /odom 이 실제보다 작음 | 엔코더 카운트 과소 | §8에서 보정 |
| /odom 부호가 반대 (직진했는데 x 가 음수) | 엔코더 방향 반대 | 펌웨어 [main.cpp:89](firmware/esp32_pid/src/main.cpp#L89)에서 부호 뒤집기 |

---

## 8. 휠 파라미터 보정 (오차 5% 이상일 때만)

**파라미터 위치 두 군데 — 둘 다 같은 값을 써야 함:**

| 위치 | 파라미터 | 현재 기본값 |
|---|---|---|
| [firmware/esp32_pid/src/main.cpp:20-22](firmware/esp32_pid/src/main.cpp#L20-L22) | `WHEEL_RADIUS`, `WHEEL_BASE`, `TICKS_PER_REV` | 0.0625, 0.542, 1200 |
| [src/robot_bringup/launch/mapping.launch.py:139-141](src/robot_bringup/launch/mapping.launch.py#L139-L141) | `wheel_radius`, `wheelbase`, `ticks_per_rev` | 동일 |

**보정 공식:**
```
실제 거리 측정값  ÷  /odom 의 x 값  =  보정 비율
```

예: 실제 1.00 m, /odom 0.87 m → 보정 비율 1.149.
- `wheel_radius`를 1.149배 → 0.0625 → 0.0718 으로 수정
- 또는 `ticks_per_rev`를 1/1.149배 → 1200 → 1044 로 수정 (정수에 가까움이면 이쪽)

**둘 중 어느 것?**
- `wheel_radius` 보정: 바퀴 실측이 부정확했을 때
- `ticks_per_rev` 보정: 엔코더 분해능 사양이 부정확했을 때 (1200이 사실 다른 값이었을 가능성)

**보정 후:**
1. `mapping.launch.py` 의 파라미터만 바꾸면 펌웨어 재업로드 불필요 (런타임 파라미터)
2. 펌웨어 값은 EKF·페일세이프에서 쓰이므로 정확도 중요하면 같이 맞춰주기
3. §7 다시 측정 → 오차 5% 안에 들 때까지 반복 (보통 2~3번 만에 수렴)

---

## 9. 직진/회전 평형 검증

**직진 평형:**
1. 1 m 직진 명령
2. 출발점·도착점 모두 마스킹테이프로 표시
3. **로봇이 좌/우로 얼마나 휘었는지** 측정
- ±5 cm 이내: OK
- 크게 휨: `wheelbase` 보정 또는 좌·우 모터 비대칭. 일단 §11 PID 튜닝에서 잡힐 수도 있음.

**회전 평형:**
1. `o` 키로 360° 제자리 회전 (텔레옵에서 angular만 명령)
2. 시각적으로 정확히 한 바퀴 도는지 + `/odom` 의 yaw 변화량이 ~2π (6.28 rad)인지
- yaw 변화량 < 2π: `wheelbase` 가 실제보다 작게 설정됨 → 키우기
- yaw 변화량 > 2π: `wheelbase` 가 실제보다 큼 → 줄이기

---

## 10. ⏭️ 첫 매핑 (작은 공간, 한 바퀴)

> §7~9가 통과 안 됐어도 매핑 자체는 가능. SLAM이 scan matching 으로 어느 정도 보정함.
> 단 결과 맵 품질은 오도메트리 정확도에 비례. **여유 있으면 §7~9부터 완료 후 진행 권장.**

**준비 — RViz2 새 터미널:**
```bash
ros2 run rviz2 rviz2
```

**RViz2 설정 (한 번만 — 저장해두면 다음부터 자동):**
1. **Fixed Frame: `map`** (좌측 상단 Global Options에서)
2. 좌측 하단 `Add` → `By topic` → `/map` → `Map` 선택
3. 추가된 Map display 펼치기 → **`Durability Policy: Transient Local`** 로 변경 ← **이거 안 하면 맵 안 보임**
4. (선택) `Add` → `LaserScan` → `/scan` (라이다 점이 빨갛게 보임)
5. (선택) `Add` → `RobotModel` → URDF 따라 로봇 모형 표시
6. **File → Save Config As** → `~/capstone/rviz/mapping.rviz` (저장해두기)

**매핑 절차:**
1. 텔레옵 창 활성
2. 천천히 (0.10 m/s 직진 / 0.3 rad/s 회전) 공간을 한 바퀴
3. **회전은 정지 후에** — 직진 중 회전하면 휠 슬립 ↑
4. 시작점으로 돌아와서 닫힌 루프 만들기 (Loop closure가 맵을 정렬해줌)
5. RViz2에서 맵이 채워지는 거 실시간 확인

**문제 발생 시:**
- 회전 시 맵이 어긋남 → 휠 슬립 또는 IMU 미융합. **EKF 통합이 필요한 신호.** (§12-A)
- 직진 시 라이다 점이 흩어짐 → 진동 또는 라이다 마운트 문제
- 맵이 전혀 안 그려짐 → §13-E

---

## 11. ⏭️ PID 튜닝 (직진이 떨리거나 가속이 어색하면)

**목적:** 바퀴 각속도 PID를 최적화해 명령 추종 품질 ↑.

**rqt_plot 설정:**
```bash
ros2 run rqt_plot rqt_plot \
  /cmd_vel/linear/x \
  /odom/twist/twist/linear/x
```

**튜닝 루프:**
1. 텔레옵으로 직진 1~2초 명령 → 정지 → 그래프 모양 확인
2. 게인 변경 (펌웨어 재업로드 ❌, 토픽 한 줄로 즉시 적용):
   ```bash
   ros2 topic pub --once /pid_gain std_msgs/Float32MultiArray "{data: [50.0, 8.0, 0.5]}"
   ```
3. serial_bridge 로그에 두 줄 떠야 적용 확인:
   ```
   PID gain sent: GAIN:50.0000,8.0000,0.5000
   ESP32 PID gains now: 50.000,8.000,0.500,12.000   ← 이게 ESP32 응답
   ```
4. 다시 직진 → 그래프 → 게인 조정 …

**진단 표:**

| 그래프 / 증상 | 원인 | 처방 |
|---|---|---|
| 출발 시 덜덜덜 | Kp 과함 | Kp 10~20씩 ↓ (현재 70) |
| 가속 느림, 늦게 따라옴 | Kp 부족 | Kp ↑ |
| 정상상태 오차 (명령보다 항상 느림) | Ki 부족 또는 PWM 데드존 | Ki ↑ (4→8→12), `integral_limit` 도 ↑ |
| 정지 명령 시 오버슈트 (휙 지나가서 보정) | Kd 부족 | Kd 1.0~2.0 시도 |
| 빠른 진동 (수 Hz) | Kd 과함 | Kd ↓ |

**현재 기본 게인 (참고):** `Kp=70, Ki=12, Kd=0.8, ilim=12`

**튜닝 완료 후 — 영구화:**
1. 만족스러운 값을 [firmware/esp32_pid/src/main.cpp:27-30](firmware/esp32_pid/src/main.cpp#L27-L30)에 박아 넣기
2. **모터 전원 OFF**한 상태에서 한 번만 `pio run -t upload` (이때 모터 의도치 않게 구동 가능)
3. 모터 전원 다시 ON, 검증

---

## 12. ⏭️ 후속 작업 (Day 2+ 또는 매핑 결과 보고 결정)

### A. EKF 통합 (회전 시 맵 어긋남 보이면)

준비된 [ekf.yaml](src/ur3_4040_urdf/config/ekf.yaml)을 mapping.launch.py에 통합:
1. `mapping.launch.py`에 `ekf_filter_node` 추가
2. `serial_bridge`의 TF 발행 OFF (EKF가 대신 발행하므로 충돌 방지)
3. `/odom`은 그대로 serial_bridge가 발행, `/imu/data`는 IMU가 발행, EKF가 둘을 융합해 `odom→base_footprint` TF + `/odometry/filtered` 발행

**효과:** IMU yaw rate 가 회전 추정에 들어가 휠 슬립 보정. 특히 제자리 회전 시 효과 큼.

작업 시간: 30분~1시간. 매핑 1차 결과 어긋남이 명확할 때 진행.

### B. Nav2 자율주행

1. 저장된 맵으로 `navigation.launch.py` 기동
2. RViz2에서 "2D Pose Estimate"로 초기 위치 → "Nav2 Goal"로 목표 지정
3. `nav2_params.yaml`은 시뮬 잔재 — 실물 맞춰 튜닝 필요 (특히 `controller_server` 의 속도 한계, costmap inflation_radius)
4. **EKF 켰으면** `nav2_params.yaml` 에서 `odom_topic: /odometry/filtered` 로 바꾸기

작업 시간: 2~3일 (튜닝 포함).

### C. 자율 탐사 (explore_lite) — 스트레치 목표

여유 있으면 시도. 데드라인 데모에 필수 아님.

---

## 13. 트러블슈팅

### A. `/dev/ttyUSB*` 안 잡힘
- USB 케이블 데이터 라인 있는지 (전원 전용 케이블 ❌)
- `lsusb` 로 디바이스 자체는 보이는지
- 권한: `sudo usermod -a -G dialout $USER` 후 재로그인
- `dmesg | tail -20` 로 커널 메시지 확인

### B. 라이다 안 잡힘 / `/scan` 안 옴
- 라파에서 `ls /dev/ttyUSB*` — 라이다 자체 잡히는지
- `ydlidar.yaml` 의 `port` 가 실제 디바이스명과 일치하는지
- 라이프사이클 노드가 activate 됐는지: `ros2 lifecycle get /ydlidar_ros2_driver_node` → `active` 떠야 함
- 라이다 모터 도는 소리 들리는지 (전원 확인)

### C. IMU 안 잡힘 / `/imu/data` 안 옴
- `i2cdetect -y 1` 에 `0x29` 또는 `0x28` 떠야 함 — 안 뜨면 배선
- ADR 핀이 HIGH면 0x29, LOW면 0x28 — 코드는 0x29 가정 ([sensors.launch.py:109](src/robot_bringup/launch/sensors.launch.py#L109))
- adafruit-circuitpython-bno055 / adafruit-blinka 설치 확인

### D. 모터가 의도치 않게 움직임
- 즉시 모터 전원 OFF
- ESP32 펌웨어 업로드 중인지 (`skip_upload:=true` 기본이라 안 일어나야 정상)
- `/cmd_vel` 발행 중인 노드 확인: `ros2 topic info /cmd_vel --verbose`
- 텔레옵 창에서 키가 stuck 됐는지

### E. SLAM 맵이 안 그려짐
- RViz Map display 의 `Durability Policy = Transient Local` 인지 (가장 흔한 원인)
- Fixed Frame 이 `map` 인지
- `ros2 topic hz /map` — 발행 자체가 되고 있는지 (보통 0.5~1 Hz)
- `ros2 run tf2_tools view_frames` 로 TF 트리 PDF 떨궈서 확인
- slam_toolbox 로그에 에러 있는지

### F. /odom 의 부호가 반대 (직진하면 x가 음수, 회전 방향 뒤집힘)
- 좌·우 엔코더 부호 → 펌웨어 [main.cpp:89](firmware/esp32_pid/src/main.cpp#L89) 참고
- 또는 모터 배선 자체가 좌↔우 바뀜 (직진 명령 시 양쪽 다 같은 방향으로 돌아야 함)

### G. ESP32 펌웨어를 새로 업로드해야 함
1. 모터 전원 OFF
2. `mapping.launch.py` 종료
3. 수동 업로드:
   ```bash
   source /home/min/.platformio/penv/bin/activate
   cd ~/capstone/firmware/esp32_pid
   pio run -t upload
   ```
4. 다시 `mapping.launch.py` 시작
5. 모터 전원 ON

---

## 14. 오늘의 성공 기준

**최소 (반드시):**
- [ ] 헬스체크 3개 모두 ✅
- [ ] 모터 전원 ON 후 의도치 않은 움직임 없음
- [ ] 텔레옵으로 로봇 직진·회전 가능
- [ ] `/odom` 의 x 값이 실제 이동 거리와 ±10% 안에서 일치

**목표 (가능하면):**
- [ ] /odom 정확도 ±5%로 보정
- [ ] PID 게인 조정으로 직진 시 떨림 없음
- [ ] 작은 공간 매핑 한 바퀴 완료, 맵 저장

**보너스 (시간 남으면):**
- [ ] 회전 평형 검증 (360° 회전)
- [ ] RViz 설정 파일 저장
- [ ] EKF 통합 시작

---

## 부록 — 자주 쓰는 명령 모음

```bash
# 빌드
cd ~/capstone && colcon build --symlink-install && source install/setup.bash

# 매핑 (PC, 라파 sensors 따로 띄운 상태)
ros2 launch robot_bringup mapping.launch.py skip_rpi:=true

# 매핑 (PC가 라파까지 SSH로 다 띄움)
ros2 launch robot_bringup mapping.launch.py

# 라파 sensors 따로
ssh danial@10.42.0.78
ws_setting capstone
ros2 launch robot_bringup sensors.launch.py

# 텔레옵
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.10 -p turn:=0.3

# rqt_plot — 속도 추종
ros2 run rqt_plot rqt_plot /cmd_vel/linear/x /odom/twist/twist/linear/x

# PID 게인 변경
ros2 topic pub --once /pid_gain std_msgs/Float32MultiArray "{data: [50.0, 8.0, 0.5]}"

# 맵 저장
ros2 run nav2_map_server map_saver_cli -f ~/capstone/src/robot_bringup/maps/my_real_map

# 토픽 진단
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames    # TF 트리 PDF
```

---

## 15. 진행 기록 (오늘 한 일 메모 — 빈 칸 채우기)

> 작업 중 막힌 거 / 보정한 값 / 새로 배운 것 메모. 다음 작업에 참고됨.

- [ ] 빌드 정상 (PC, 라파):
- [ ] /dev/ttyUSB 잡힘 — PC:           라파:
- [ ] 헬스체크 통과 시각:
- [ ] /odom 1m 직진 결과 — 실제:           cm, /odom:           cm
- [ ] 보정 후 wheel_radius / ticks_per_rev:
- [ ] PID 게인 최종값: Kp=        Ki=        Kd=
- [ ] 첫 매핑 결과 (잘된 점 / 어긋난 점):
- [ ] 내일 이어서 할 일:

화이팅 🚀
