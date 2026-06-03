# esp32_pid — ESP32 모터 PID 펌웨어

차동구동 베이스의 좌·우 바퀴에 PID 속도 제어를 거는 ESP32 펌웨어. PC 쪽 `ros2_pid_bridge`가
USB 시리얼로 `CMD_VEL`을 보내면 펌웨어가 휠 목표 각속도로 환산해 PWM을 만들고, 엔코더 카운트와
측정 휠 각속도를 `FB:` 라인으로 되돌려보낸다.

## 디렉토리 구성

```
firmware/esp32_pid/
├── platformio.ini    PlatformIO 빌드 설정 (board=esp32dev, framework=arduino, 115200)
└── src/main.cpp      펌웨어 본체
```

## 빌드 & 업로드

PlatformIO CLI는 `/home/min/.platformio/penv/`의 가상환경에 설치돼 있다.

```bash
source /home/min/.platformio/penv/bin/activate
cd /home/min/capstone/firmware/esp32_pid
pio run -t upload
```

`pio`는 `/dev/ttyUSB0`을 자동 감지한다. 업로드가 끝나면 ESP32가 자동 리셋되고,
부팅 직후 `ESP32 ROS2 PID motor controller starting` 배너와 현재 PID 게인이 시리얼에 찍힌다.

펌웨어 동작만 빠르게 보고 싶을 때:

```bash
pio device monitor -b 115200
```

(주의: `ros2_pid_bridge` 노드와 시리얼 포트를 동시에 잡을 수는 없다 — 한쪽을 끄고 사용)

## ESP32 핀맵

| 기능 | GPIO |
|---|---|
| Left PWM (AN1)  | 25 |
| Right PWM (AN2) | 26 |
| Left Direction (IN1)  | 27 |
| Right Direction (IN2) | 14 |
| Left Encoder A / B  | 34 / 35 |
| Right Encoder A / B | 32 / 33 |

엔코더 핀은 모두 `INPUT_PULLUP`. 4채널 모두 `CHANGE` 인터럽트로 쿼드러처 디코딩한다.
오른쪽 엔코더는 좌측과 부호가 반대로 잡혀, +PWM 전진 시 양쪽 카운트가 모두 +로 증가하도록
ISR에서 부호를 뒤집어 둔다.

## 제어 파라미터 (펌웨어 컴파일 시 고정)

- `WHEEL_RADIUS = 0.0625 m` (외경 125mm)
- `WHEEL_BASE   = 0.542 m` (URDF 기준 0.271×2 — 실측 후 보정 권장)
- `TICKS_PER_REV = 1200`
- 제어 주기 20 ms, 피드백 송신 50 ms 주기
- PID 초기값 `Kp=70, Ki=12, Kd=0.8, ilim=12`
- **명령 워치독**: `CMD_VEL`/`CMD`/`PWM` 어느 것도 500 ms 이상 안 들어오면 모터를 끄고 PID 적분 상태까지 리셋

## 시리얼 프로토콜 (115200 8N1)

PC → ESP32 (모두 `\n` 종결, `\r`은 무시):

| 명령 | 의미 |
|---|---|
| `CMD_VEL:<v_mm/s>,<w_mrad/s>` | 트위스트. 펌웨어가 휠 목표 각속도로 환산해 PID로 추종 (`ros2_pid_bridge`가 보내는 형식) |
| `CMD:<vL_mrad/s>,<vR_mrad/s>` | 좌·우 휠 각속도 직접 지정 (PID 사용) |
| `PWM:<left>,<right>` | **PID 우회**, PWM(-255~255) 직접 인가. 모터/엔코더 단독 진단용 |
| `GAIN:<kp>,<ki>,<kd>` 또는 `GAIN:<kp>,<ki>,<kd>,<ilim>` | PID 게인 런타임 변경. 적용 후 적분/미분 상태 리셋 |
| `GAIN?` | 현재 게인 출력 |
| `RAW` | 엔코더 A/B 핀 raw 상태를 매 제어주기마다 출력 (배선 점검용) |

ESP32 → PC:

| 라인 | 의미 |
|---|---|
| `FB:<encL>,<encR>,<velL_mrad/s>,<velR_mrad/s>` | 50 ms 주기 피드백. enc는 누적 카운트, vel은 최근 주기 측정값 |
| `GAIN_SET:<kp>,<ki>,<kd>,<ilim>` | 부팅 시 및 `GAIN:` 적용 후 |
| `RAW:<R_A>,<R_B>,<L_A>,<L_B>` | `RAW` 명령이 활성화된 동안 매 주기 |

## 진단 절차 (모터·엔코더 안 도는 것 같을 때)

1. `pio device monitor`로 부팅 배너가 뜨는지 확인 — 펌웨어가 살아있는지
2. `PWM:120,120\n` 송신 → 모터가 도는지 (PID 무관, 모터 드라이버 라인 점검)
3. `RAW\n` 송신 후 손으로 바퀴 굴리기 → `RAW:` 출력의 A/B가 토글되는지 (엔코더 신호 도달 점검)
4. 정상이면 `CMD_VEL:80,0\n` → 전진 + 양쪽 enc가 +로 증가해야 함
