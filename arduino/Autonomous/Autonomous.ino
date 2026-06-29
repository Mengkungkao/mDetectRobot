/*
  AutonomousV11 low-level controller for Arduino Uno

  Responsibilities:
    - Read four QGPMaker quadrature encoders
    - Read MPU6050 yaw and yaw rate
    - Run four independent wheel-speed PID controllers
    - Drive four motors through the QGPMaker motor shield
    - Enforce command watchdog and latched emergency stop
    - Brake for a configurable time, then release without blocking

  Serial protocol at 500000 baud
  Pi -> Arduino:
    V,seq,m1,m2,m3,m4       wheel targets in mm/s
    B,seq,hold_ms            brake, then release
    S,seq                    normal stop: brake 1 s, then release
    E,seq                    latch emergency stop
    C,seq                    clear emergency stop
    R,seq                    reset encoder counts and yaw zero
    P,seq,motor,kp,ki,kd     update PID gains, motor is 1..4
    Q,seq                    request immediate telemetry

  Arduino -> Pi:
    T,seq,millis,e1,e2,e3,e4,v1,v2,v3,v4,yaw_cdeg,gyro_cdeg_s,state,fault
    A,seq,code
    READY
*/

#include <Wire.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"
#include <MPU6050_tockn.h>

// ---------------- Hardware and timing ----------------

const uint32_t SERIAL_BAUD = 500000UL;
const uint8_t MOTOR_COUNT = 4;
const uint16_t CONTROL_PERIOD_MS = 20;      // 50 Hz
const uint16_t TELEMETRY_PERIOD_MS = 20;    // 50 Hz
const uint16_t COMMAND_WATCHDOG_MS = 250;
const uint16_t DEFAULT_BRAKE_HOLD_MS = 1000;
const uint8_t LINE_BUFFER_SIZE = 96;

const float WHEEL_DIAMETER_MM = 80.5f;
const float COUNTS_PER_REV = 4320.0f;
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;
const float MAX_TARGET_SPEED_MM_S = 300.0f;

const uint8_t MIN_MOVING_PWM = 45;
const uint8_t MAX_PWM = 220;

// Existing robot calibration. Tune after verifying all encoder signs and directions.
const float MOTOR_FEEDFORWARD_SCALE[MOTOR_COUNT] PROGMEM = {
  0.9069f, 1.0630f, 1.0987f, 0.6170f
};

// Makes corrected encoder counts positive when the robot drives forward.
const int8_t ENCODER_SIGN[MOTOR_COUNT] PROGMEM = { 1, 1, 1, 1 };

// Set an entry to -1 if that motor turns opposite to the requested direction.
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] PROGMEM = { -1, 1, 1, -1 };

// Set to -1.0 if positive ROS yaw appears clockwise in RViz.
const float IMU_YAW_SIGN = 1.0f;

QGPMaker_MotorShield AFMS;
QGPMaker_Encoder enc1(1), enc2(2), enc3(3), enc4(4);
QGPMaker_DCMotor *motors[MOTOR_COUNT];
MPU6050 mpu6050(Wire);

// ---------------- Controller types ----------------

struct SpeedPID {
  float kp;
  float ki;
  float kd;
  float integral;
  float previousError;

  void configure(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
    reset();
  }

  void reset() {
    integral = 0.0f;
    previousError = 0.0f;
  }

  float update(float target, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;
    const float error = target - measured;
    integral += error * dt;
    integral = constrain(integral, -800.0f, 800.0f);
    const float derivative = (error - previousError) / dt;
    previousError = error;
    return kp * error + ki * integral + kd * derivative;
  }
};

struct ScalarKalman {
  float x;
  float p;
  bool ready;

  void reset() {
    x = 0.0f;
    p = 1.0f;
    ready = false;
  }

  float update(float measurement) {
    const float q = 30.0f;
    const float r = 180.0f;

    if (!ready) {
      x = measurement;
      p = 1.0f;
      ready = true;
      return x;
    }

    // A large jump usually means a torn/noisy sample. Limit rather than discard forever.
    const float innovation = constrain(measurement - x, -500.0f, 500.0f);
    p += q;
    const float k = p / (p + r);
    x += k * innovation;
    p *= (1.0f - k);
    return x;
  }
};

enum ControllerState : uint8_t {
  STATE_RELEASED = 0,
  STATE_RUNNING = 1,
  STATE_BRAKING = 2,
  STATE_ESTOP = 3,
  STATE_WATCHDOG = 4,
  STATE_FAULT = 5
};

enum FaultBits : uint8_t {
  FAULT_NONE = 0,
  FAULT_WATCHDOG = 1 << 0,
  FAULT_ESTOP = 1 << 1,
  FAULT_PARSE = 1 << 2
};

SpeedPID pid[MOTOR_COUNT];
ScalarKalman speedFilter[MOTOR_COUNT];

float targetSpeedMMs[MOTOR_COUNT] = {0, 0, 0, 0};
float measuredSpeedMMs[MOTOR_COUNT] = {0, 0, 0, 0};
int32_t correctedEncoderCounts[MOTOR_COUNT] = {0, 0, 0, 0};
int32_t previousEncoderCounts[MOTOR_COUNT] = {0, 0, 0, 0};

uint16_t lastSequence = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastValidCommandMs = 0;
uint32_t brakeUntilMs = 0;

bool commandSeen = false;
bool brakeActive = false;
bool estopLatched = false;
uint8_t stateCode = STATE_RELEASED;
uint8_t faultCode = FAULT_NONE;

float yawZeroRawDeg = 0.0f;
char serialLine[LINE_BUFFER_SIZE];
uint8_t serialIndex = 0;

// ---------------- Utility ----------------

bool timeReached(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

float normalizeDegrees(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float getRosYawDeg() {
  return normalizeDegrees((mpu6050.getAngleZ() - yawZeroRawDeg) * IMU_YAW_SIGN);
}

float getRosGyroDegS() {
  return mpu6050.getGyroZ() * IMU_YAW_SIGN;
}

bool targetsAreZero() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (fabs(targetSpeedMMs[i]) > 0.5f) return false;
  }
  return true;
}

void resetControllers() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    pid[i].reset();
    speedFilter[i].reset();
    measuredSpeedMMs[i] = 0.0f;
  }
}

void releaseMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    motors[i]->setSpeed(0);
    motors[i]->run(RELEASE);
  }
}

void applyBrake() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    motors[i]->setSpeed(0);
    motors[i]->run(BRAKE);
  }
}

void setTargetsZero() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) targetSpeedMMs[i] = 0.0f;
}

void beginBrake(uint16_t holdMs, uint8_t stopState) {
  setTargetsZero();
  resetControllers();
  applyBrake();
  brakeActive = true;
  brakeUntilMs = millis() + holdMs;
  stateCode = stopState;
}

void updateBrakeState(uint32_t now) {
  if (!brakeActive || !timeReached(now, brakeUntilMs)) return;
  brakeActive = false;
  releaseMotors();

  if (estopLatched) stateCode = STATE_ESTOP;
  else if (faultCode & FAULT_WATCHDOG) stateCode = STATE_WATCHDOG;
  else stateCode = STATE_RELEASED;
}

void readEncoderCountsAtomic() {
  int32_t raw[MOTOR_COUNT];
  noInterrupts();
  raw[0] = enc1.read();
  raw[1] = enc2.read();
  raw[2] = enc3.read();
  raw[3] = enc4.read();
  interrupts();

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const int8_t sign = (int8_t)pgm_read_byte(&ENCODER_SIGN[i]);
    correctedEncoderCounts[i] = raw[i] * sign;
  }
}

void resetSensorReference() {
  noInterrupts();
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  enc4.write(0);
  interrupts();

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    correctedEncoderCounts[i] = 0;
    previousEncoderCounts[i] = 0;
  }
  yawZeroRawDeg = mpu6050.getAngleZ();
  resetControllers();
}

float feedForwardPWM(float targetAbs, uint8_t motorIndex) {
  if (targetAbs < 0.5f) return 0.0f;
  const float ratio = constrain(targetAbs / MAX_TARGET_SPEED_MM_S, 0.0f, 1.0f);
  float pwm = MIN_MOVING_PWM + ratio * (MAX_PWM - MIN_MOVING_PWM);
  pwm *= pgm_read_float(&MOTOR_FEEDFORWARD_SCALE[motorIndex]);
  return constrain(pwm, 0.0f, 255.0f);
}

void driveMotorSigned(uint8_t index, float signedPWM) {
  signedPWM *= (int8_t)pgm_read_byte(&MOTOR_DIRECTION_SIGN[index]);
  const uint8_t pwm = (uint8_t)constrain((int)(fabs(signedPWM) + 0.5f), 0, 255);
  if (pwm == 0) {
    motors[index]->setSpeed(0);
    motors[index]->run(RELEASE);
    return;
  }

  motors[index]->setSpeed(pwm);
  motors[index]->run(signedPWM >= 0.0f ? FORWARD : BACKWARD);
}

void runSpeedControl(float dt) {
  if (brakeActive || estopLatched || (faultCode & FAULT_WATCHDOG)) return;

  if (targetsAreZero()) {
    releaseMotors();
    resetControllers();
    stateCode = STATE_RELEASED;
    return;
  }

  stateCode = STATE_RUNNING;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const float target = constrain(targetSpeedMMs[i], -MAX_TARGET_SPEED_MM_S, MAX_TARGET_SPEED_MM_S);
    if (fabs(target) < 0.5f) {
      driveMotorSigned(i, 0.0f);
      pid[i].reset();
      continue;
    }

    const float sign = target >= 0.0f ? 1.0f : -1.0f;
    const float ff = sign * feedForwardPWM(fabs(target), i);
    const float trim = pid[i].update(target, measuredSpeedMMs[i], dt);
    const float output = constrain(ff + trim, -255.0f, 255.0f);
    driveMotorSigned(i, output);
  }
}

void updateSpeedMeasurement(float dt) {
  readEncoderCountsAtomic();
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const int32_t delta = correctedEncoderCounts[i] - previousEncoderCounts[i];
    previousEncoderCounts[i] = correctedEncoderCounts[i];
    const float rawSpeed = (delta * MM_PER_COUNT) / dt;
    measuredSpeedMMs[i] = speedFilter[i].update(rawSpeed);
  }
}

void sendAck(uint16_t sequence, uint8_t code) {
  Serial.print(F("A,"));
  Serial.print(sequence);
  Serial.print(',');
  Serial.println(code);
}

void sendTelemetry() {
  readEncoderCountsAtomic();
  Serial.print(F("T,"));
  Serial.print(lastSequence);
  Serial.print(',');
  Serial.print(millis());

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(',');
    Serial.print(correctedEncoderCounts[i]);
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(',');
    Serial.print((int)round(measuredSpeedMMs[i]));
  }

  Serial.print(',');
  Serial.print((int)round(getRosYawDeg() * 100.0f));
  Serial.print(',');
  Serial.print((int)round(getRosGyroDegS() * 100.0f));
  Serial.print(',');
  Serial.print(stateCode);
  Serial.print(',');
  Serial.println(faultCode);
}

char *nextToken(char **context) {
  return strtok_r(NULL, ",", context);
}

bool parseLongToken(char **context, long &value) {
  char *token = nextToken(context);
  if (token == NULL) return false;
  char *endPtr = NULL;
  value = strtol(token, &endPtr, 10);
  return endPtr != token && *endPtr == '\0';
}

bool parseFloatToken(char **context, float &value) {
  char *token = nextToken(context);
  if (token == NULL) return false;
  char *endPtr = NULL;
  value = strtod(token, &endPtr);
  return endPtr != token && *endPtr == '\0';
}

void handleVelocityCommand(char **context) {
  long seq;
  if (!parseLongToken(context, seq)) {
    faultCode |= FAULT_PARSE;
    return;
  }

  float incoming[MOTOR_COUNT];
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (!parseFloatToken(context, incoming[i])) {
      faultCode |= FAULT_PARSE;
      return;
    }
  }

  lastSequence = (uint16_t)seq;
  commandSeen = true;
  lastValidCommandMs = millis();

  if (estopLatched) {
    sendAck(lastSequence, 2);  // rejected because ESTOP is latched
    return;
  }

  faultCode &= ~FAULT_WATCHDOG;
  faultCode &= ~FAULT_PARSE;

  bool incomingIsZero = true;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    targetSpeedMMs[i] = constrain(incoming[i], -MAX_TARGET_SPEED_MM_S, MAX_TARGET_SPEED_MM_S);
    if (fabs(targetSpeedMMs[i]) > 0.5f) incomingIsZero = false;
  }
  // Continuous zero commands from the Pi must not cancel a requested brake hold.
  if (!incomingIsZero) brakeActive = false;
  sendAck(lastSequence, 0);
}

void handleLine(char *line) {
  char *context = NULL;
  char *command = strtok_r(line, ",", &context);
  if (command == NULL) return;

  if (strcmp(command, "V") == 0) {
    handleVelocityCommand(&context);
    return;
  }

  long seqLong = 0;
  if (!parseLongToken(&context, seqLong)) {
    faultCode |= FAULT_PARSE;
    return;
  }
  lastSequence = (uint16_t)seqLong;

  if (strcmp(command, "B") == 0) {
    long hold = DEFAULT_BRAKE_HOLD_MS;
    char *holdToken = nextToken(&context);
    if (holdToken != NULL) hold = strtol(holdToken, NULL, 10);
    hold = constrain(hold, 50L, 5000L);
    beginBrake((uint16_t)hold, STATE_BRAKING);
    lastValidCommandMs = millis();
    commandSeen = true;
    sendAck(lastSequence, 0);
  }
  else if (strcmp(command, "S") == 0) {
    beginBrake(DEFAULT_BRAKE_HOLD_MS, STATE_BRAKING);
    lastValidCommandMs = millis();
    commandSeen = true;
    sendAck(lastSequence, 0);
  }
  else if (strcmp(command, "E") == 0) {
    estopLatched = true;
    faultCode |= FAULT_ESTOP;
    beginBrake(DEFAULT_BRAKE_HOLD_MS, STATE_ESTOP);
    sendAck(lastSequence, 0);
  }
  else if (strcmp(command, "C") == 0) {
    setTargetsZero();
    estopLatched = false;
    faultCode &= ~FAULT_ESTOP;
    faultCode &= ~FAULT_WATCHDOG;
    brakeActive = false;
    releaseMotors();
    stateCode = STATE_RELEASED;
    commandSeen = false;
    sendAck(lastSequence, 0);
  }
  else if (strcmp(command, "R") == 0) {
    setTargetsZero();
    releaseMotors();
    resetSensorReference();
    faultCode &= ~FAULT_WATCHDOG;
    commandSeen = false;
    stateCode = estopLatched ? STATE_ESTOP : STATE_RELEASED;
    sendAck(lastSequence, 0);
  }
  else if (strcmp(command, "P") == 0) {
    long motorNumber;
    float kp, ki, kd;
    if (!parseLongToken(&context, motorNumber) ||
        !parseFloatToken(&context, kp) ||
        !parseFloatToken(&context, ki) ||
        !parseFloatToken(&context, kd) ||
        motorNumber < 1 || motorNumber > 4) {
      faultCode |= FAULT_PARSE;
      sendAck(lastSequence, 1);
      return;
    }
    pid[motorNumber - 1].configure(kp, ki, kd);
    sendAck(lastSequence, 0);
  }
  else if (strcmp(command, "Q") == 0) {
    sendTelemetry();
  }
  else {
    faultCode |= FAULT_PARSE;
    sendAck(lastSequence, 1);
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ':') {
      if (serialIndex == 0) continue;
      serialLine[serialIndex] = '\0';
      serialIndex = 0;
      handleLine(serialLine);
      continue;
    }

    if (serialIndex < LINE_BUFFER_SIZE - 1) {
      serialLine[serialIndex++] = c;
    } else {
      serialIndex = 0;
      faultCode |= FAULT_PARSE;
    }
  }
}

void checkWatchdog(uint32_t now) {
  if (!commandSeen || estopLatched) return;
  if ((uint32_t)(now - lastValidCommandMs) <= COMMAND_WATCHDOG_MS) return;

  commandSeen = false;
  faultCode |= FAULT_WATCHDOG;
  beginBrake(DEFAULT_BRAKE_HOLD_MS, STATE_WATCHDOG);
}

void setupPID() {
  pid[0].configure(0.25f, 0.034f, 0.003f);
  pid[1].configure(0.239f, 0.034f, 0.003f);
  pid[2].configure(0.23f, 0.015f, 0.003f);
  pid[3].configure(0.25f, 0.034f, 0.003f);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000UL);
  AFMS.begin(500);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    motors[i] = AFMS.getMotor(i + 1);
  }

  releaseMotors();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false, 1000, 1000);
  mpu6050.update();
  yawZeroRawDeg = mpu6050.getAngleZ();

  setupPID();
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) speedFilter[i].reset();
  readEncoderCountsAtomic();
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) previousEncoderCounts[i] = correctedEncoderCounts[i];

  lastControlMs = millis();
  lastTelemetryMs = millis();
  Serial.println(F("READY"));
}

void loop() {
  mpu6050.update();
  readSerialCommands();

  const uint32_t now = millis();
  checkWatchdog(now);
  updateBrakeState(now);

  if ((uint32_t)(now - lastControlMs) >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlMs) * 0.001f;
    lastControlMs = now;
    dt = constrain(dt, 0.005f, 0.100f);
    updateSpeedMeasurement(dt);
    runSpeedControl(dt);
  }

  if ((uint32_t)(now - lastTelemetryMs) >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}
