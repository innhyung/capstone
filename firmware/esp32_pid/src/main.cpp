#include <Arduino.h>

// Motor driver pins
static const int AN1_PIN = 25; // Left PWM
static const int AN2_PIN = 26; // Right PWM
static const int IN1_PIN = 27; // Left direction
static const int IN2_PIN = 14; // Right direction

// Encoder pins
static const int ENC_L_A = 34;
static const int ENC_L_B = 35;
static const int ENC_R_A = 32;
static const int ENC_R_B = 33;

const int LEFT_CHANNEL = 0;
const int RIGHT_CHANNEL = 1;
const int PWM_FREQ = 20000;
const int PWM_RES = 8; // 0-255

const float WHEEL_RADIUS = 0.0625f; // 반지름 (외경 125mm / 2)
const float WHEEL_BASE = 0.58f;    // 자동 회전 테스트 보정값 (720° → 765° 실제, 0.62 × 720/765)
const float TICKS_PER_REV = 11400.0f;  // 19 PPR × 4 quad × 150 gear (IG-52GM).
                                       // 2026-06-03: 왼쪽 엔코더 B상 배선 복구 → 풀 ×4 디코딩 회복.
                                       // 실측 1m → odom 2.0m(=2배)로 5700이 과소 → 이론값 11400 복원.
const float CONTROL_PERIOD = 0.020f; // 20 ms
const float PI2 = 6.28318530718f;

// PID 게인 — 런타임에 GAIN: 시리얼 명령으로 조정 가능 (튜닝용)
// 2026-06-03 회전 떨림 튜닝: 양쪽 엔코더 복구 후 기존 70/12/0.8 이 진동 유발.
// Kp 낮추고 Kd=0 (미분항이 노이즈 증폭) → 회전 매끄러움.
float pidKp = 30.0f;
float pidKi = 6.0f;
float pidKd = 0.0f;
float pidIntegralLimit = 12.0f; // 적분 기여 PWM을 ~KI*ilim=144로 제한 (windup 방지)

volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;
volatile uint8_t leftPrevState = 0;
volatile uint8_t rightPrevState = 0;

float targetLeftVel = 0.0f;
float targetRightVel = 0.0f;
float errorSumLeft = 0.0f;
float errorSumRight = 0.0f;
float prevErrorLeft = 0.0f;
float prevErrorRight = 0.0f;

// 오픈루프 PWM 디버그 모드 (PID 우회 — 모터/엔코더 테스트용)
bool openLoop = false;
int openLoopLeft = 0;
int openLoopRight = 0;

// [임시 진단] 엔코더 A/B 핀 raw 상태 출력 모드
bool rawDebug = false;

int32_t prevLeftCount = 0;
int32_t prevRightCount = 0;
uint32_t lastControlMillis = 0;
uint32_t lastFeedbackMillis = 0;
uint32_t lastCommandMillis = 0;

char rxLine[128];
uint8_t rxPos = 0;

inline uint8_t readEncoderState(int pinA, int pinB) {
  return (digitalRead(pinA) << 1) | digitalRead(pinB);
}

void IRAM_ATTR updateLeftEncoder() {
  uint8_t currentState = readEncoderState(ENC_L_A, ENC_L_B);
  uint8_t index = (leftPrevState << 2) | currentState;
  static const int8_t quadTable[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };
  leftEncoderCount += quadTable[index];
  leftPrevState = currentState;
}

void IRAM_ATTR updateRightEncoder() {
  uint8_t currentState = readEncoderState(ENC_R_A, ENC_R_B);
  uint8_t index = (rightPrevState << 2) | currentState;
  static const int8_t quadTable[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };
  // 오른쪽 엔코더는 좌측과 부호가 반대로 잡혀 카운트 방향을 뒤집는다
  // (+PWM 전진 시 카운트가 +가 되도록 — PID/odometry 부호 일치)
  rightEncoderCount -= quadTable[index];
  rightPrevState = currentState;
}

void setMotor(int motor, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (motor == 0) {
    digitalWrite(IN1_PIN, pwm >= 0 ? HIGH : LOW);
    ledcWrite(LEFT_CHANNEL, abs(pwm));
  } else {
    digitalWrite(IN2_PIN, pwm >= 0 ? HIGH : LOW);
    ledcWrite(RIGHT_CHANNEL, abs(pwm));
  }
}

float ticksToRadPerSec(int32_t ticks, float dt) {
  float rev = ticks / TICKS_PER_REV;
  return rev * PI2 / dt;
}

float pidCompute(float target, float current, float &errorSum, float &prevError, float dt) {
  float error = target - current;
  errorSum += error * dt;
  errorSum = constrain(errorSum, -pidIntegralLimit, pidIntegralLimit);
  float derivative = (error - prevError) / dt;
  prevError = error;
  float output = pidKp * error + pidKi * errorSum + pidKd * derivative;
  return constrain(output, -255.0f, 255.0f);
}

void sendFeedback(float leftVel, float rightVel) {
  int leftVelInt = (int)round(leftVel * 1000.0f);
  int rightVelInt = (int)round(rightVel * 1000.0f);
  int32_t leftCount;
  int32_t rightCount;
  noInterrupts();
  leftCount = leftEncoderCount;
  rightCount = rightEncoderCount;
  interrupts();
  Serial.printf("FB:%ld,%ld,%d,%d\n", leftCount, rightCount, leftVelInt, rightVelInt);
}

void updateTargetFromTwist(int32_t linear_mms, int32_t angular_mradss) {
  float v = linear_mms / 1000.0f;   // m/s
  float w = angular_mradss / 1000.0f; // rad/s
  targetLeftVel  = (v - w * WHEEL_BASE * 0.5f) / WHEEL_RADIUS;
  targetRightVel = (v + w * WHEEL_BASE * 0.5f) / WHEEL_RADIUS;
}

void printGains() {
  Serial.printf("GAIN_SET:%.3f,%.3f,%.3f,%.3f\n",
                pidKp, pidKi, pidKd, pidIntegralLimit);
}

void processLine(const char *line) {
  if (strncmp(line, "CMD_VEL:", 8) == 0) {
    int32_t lin = 0, ang = 0;
    if (sscanf(line + 8, "%ld,%ld", &lin, &ang) == 2) {
      openLoop = false;
      updateTargetFromTwist(lin, ang);
      lastCommandMillis = millis();
    }
  } else if (strncmp(line, "CMD:", 4) == 0) {
    int32_t left = 0, right = 0;
    if (sscanf(line + 4, "%ld,%ld", &left, &right) == 2) {
      openLoop = false;
      targetLeftVel = left / 1000.0f;
      targetRightVel = right / 1000.0f;
      lastCommandMillis = millis();
    }
  } else if (strncmp(line, "PWM:", 4) == 0) {
    // 오픈루프 디버그: PID 없이 모터에 PWM 직접 인가 (-255~255)
    int32_t left = 0, right = 0;
    if (sscanf(line + 4, "%ld,%ld", &left, &right) == 2) {
      openLoop = true;
      openLoopLeft = constrain((int)left, -255, 255);
      openLoopRight = constrain((int)right, -255, 255);
      lastCommandMillis = millis();
    }
  } else if (strncmp(line, "GAIN:", 5) == 0) {
    // PID 게인 런타임 조정: GAIN:kp,ki,kd  또는  GAIN:kp,ki,kd,ilim
    float kp = 0, ki = 0, kd = 0, ilim = 0;
    int n = sscanf(line + 5, "%f,%f,%f,%f", &kp, &ki, &kd, &ilim);
    if (n >= 3) {
      pidKp = kp;
      pidKi = ki;
      pidKd = kd;
      if (n == 4) {
        pidIntegralLimit = ilim;
      }
      // 게인이 바뀌면 적분/미분 상태를 리셋해 출력이 튀는 것 방지
      errorSumLeft = 0.0f;
      errorSumRight = 0.0f;
      prevErrorLeft = 0.0f;
      prevErrorRight = 0.0f;
      printGains();
    }
  } else if (strncmp(line, "GAIN?", 5) == 0) {
    // 현재 PID 게인 조회
    printGains();
  } else if (strncmp(line, "RAW", 3) == 0) {
    // [임시 진단] 엔코더 A/B raw 핀 상태 출력 켜기
    rawDebug = true;
  }
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      rxLine[rxPos] = '\0';
      if (rxPos > 0) {
        processLine(rxLine);
      }
      rxPos = 0;
      continue;
    }
    if (rxPos + 1 < sizeof(rxLine)) {
      rxLine[rxPos++] = c;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println("ESP32 ROS2 PID motor controller starting");
  printGains();

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  ledcSetup(LEFT_CHANNEL, PWM_FREQ, PWM_RES);
  ledcSetup(RIGHT_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(AN1_PIN, LEFT_CHANNEL);
  ledcAttachPin(AN2_PIN, RIGHT_CHANNEL);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  leftPrevState  = readEncoderState(ENC_L_A, ENC_L_B);
  rightPrevState = readEncoderState(ENC_R_A, ENC_R_B);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), updateRightEncoder, CHANGE);

  prevLeftCount = leftEncoderCount;
  prevRightCount = rightEncoderCount;
  lastControlMillis = millis();
  lastFeedbackMillis = millis();
  lastCommandMillis = millis();
}

void loop() {
  readSerialCommands();

  uint32_t now = millis();
  if (now - lastControlMillis >= 20) {
    float dt = (now - lastControlMillis) / 1000.0f;
    lastControlMillis = now;

    int32_t leftCount;
    int32_t rightCount;
    noInterrupts();
    leftCount = leftEncoderCount;
    rightCount = rightEncoderCount;
    interrupts();

    int32_t deltaLeft = leftCount - prevLeftCount;
    int32_t deltaRight = rightCount - prevRightCount;
    prevLeftCount  = leftCount;
    prevRightCount = rightCount;

    float leftVel = ticksToRadPerSec(deltaLeft, dt);
    float rightVel = ticksToRadPerSec(deltaRight, dt);

    if (now - lastCommandMillis > 500) {
      // 페일세이프: 명령이 끊기면 PID에 의존하지 말고 모터를 직접 끄고
      // PID 적분 상태까지 모두 리셋한다 (windup이 워치독을 무력화하는 것 방지)
      targetLeftVel = 0.0f;
      targetRightVel = 0.0f;
      openLoopLeft = 0;
      openLoopRight = 0;
      errorSumLeft = 0.0f;
      errorSumRight = 0.0f;
      prevErrorLeft = 0.0f;
      prevErrorRight = 0.0f;
      setMotor(0, 0);
      setMotor(1, 0);
    } else if (openLoop) {
      // 오픈루프: PWM 직접 인가, PID 상태는 리셋해 둠 (복귀 시 windup 방지)
      setMotor(0, openLoopLeft);
      setMotor(1, openLoopRight);
      errorSumLeft = 0.0f;
      errorSumRight = 0.0f;
      prevErrorLeft = 0.0f;
      prevErrorRight = 0.0f;
    } else {
      float pwmLeft  = pidCompute(targetLeftVel, leftVel, errorSumLeft, prevErrorLeft, dt);
      float pwmRight = pidCompute(targetRightVel, rightVel, errorSumRight, prevErrorRight, dt);
      setMotor(0, (int)round(pwmLeft));
      setMotor(1, (int)round(pwmRight));
    }

    if (now - lastFeedbackMillis >= 50) {
      sendFeedback(leftVel, rightVel);
      lastFeedbackMillis = now;
    }

    if (rawDebug) {
      // [임시 진단] RAW:오른A,오른B,왼A,왼B
      Serial.printf("RAW:%d,%d,%d,%d\n",
                    digitalRead(ENC_R_A), digitalRead(ENC_R_B),
                    digitalRead(ENC_L_A), digitalRead(ENC_L_B));
    }
  }
}
