/*
  mDetect ROS2 low-level base controller

  Hardware:
  - Arduino Uno
  - QGPMaker motor shield with four DC motors
  - Four QGPMaker quadrature encoders
  - MPU6050 on I2C

  Responsibility split:
  - Arduino: 100 Hz encoder/IMU acquisition, four independent wheel-speed PID
    loops, odometry integration, command watchdog and latched emergency stop.
  - Raspberry Pi: ROS2 serial bridge, /cmd_vel, /odom, /imu/data, /joint_states.
  - Ubuntu workstation: RViz, SLAM Toolbox, Nav2, costmaps, planner and
    Regulated Pure Pursuit controller.

  ROS coordinate convention:
  - +X forward
  - +Y left
  - +yaw counter-clockwise

  Serial settings:
  - 500000 baud, 8-N-1
  - Commands are ASCII lines terminated by \n, \r or ':'

  Main command:
    VEL,<linear_mm_s>,<angular_deg_s>

  Test aliases:
    FORWARD,<speed_mm_s>
    REVERSE,<speed_mm_s>
    LEFT,<angular_deg_s>
    RIGHT,<angular_deg_s>
    STOP
    ESTOP
    CLEAR_ESTOP
    RESET_ODOM
    ZERO_YAW
    CAL_IMU
    PID,<kp>,<ki>,<kd>
    PIDM,<motor_1_to_4>,<kp>,<ki>,<kd>
    TRIM,<motor_1_to_4>,<scale>     both directions
    TRIMF,<motor_1_to_4>,<scale>    forward only
    TRIMR,<motor_1_to_4>,<scale>    reverse only
    STRAIGHT_ON
    STRAIGHT_OFF
    STRAIGHT,<kp>,<yaw_rate_kd>,<max_correction_deg_s>
    PING

  Telemetry at 20 Hz:
    T,time_ms,x_mm,y_mm,yaw_deg,vx_mm_s,wz_deg_s,
      tick1,tick2,tick3,tick4,
      speed1,speed2,speed3,speed4,
      pwm1,pwm2,pwm3,pwm4,estop,watchdog
*/

#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"
#include <MPU6050_tockn.h>

// -----------------------------------------------------------------------------
// Robot geometry and timing
// -----------------------------------------------------------------------------

const uint8_t MOTOR_COUNT = 4;
const uint32_t SERIAL_BAUD = 500000UL;
const uint16_t CONTROL_PERIOD_MS = 10;      // 100 Hz motor control
const uint16_t TELEMETRY_PERIOD_MS = 50;    // 20 Hz telemetry
const uint16_t COMMAND_WATCHDOG_MS = 500;   // stop if Pi heartbeat disappears
const uint16_t STOP_BRAKE_DURATION_MS = 1000; // brake for 1 s, then release

const float WHEEL_DIAMETER_MM = 80.5f;
const float WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM * 0.5f;
const float COUNTS_PER_REV = 4320.0f;
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;
const float TRACK_WIDTH_MM = 210.0f;        // left-to-right wheel centre distance

// Limits should stay conservative until the robot has been tested on blocks.
const float MAX_LINEAR_MM_S = 250.0f;
const float MAX_ANGULAR_DEG_S = 120.0f;
const float MAX_WHEEL_MM_S = 350.0f;
const float LINEAR_ACCEL_MM_S2 = 350.0f;
const float ANGULAR_ACCEL_DEG_S2 = 220.0f;

const uint8_t MIN_EFFECTIVE_PWM = 48;
const uint8_t MAX_DRIVE_PWM = 210;

// Motor direction calibration.
// The QGPMaker motors are mounted opposite to the library's electrical
// FORWARD direction on this robot. A ROS-positive wheel target therefore needs
// a physical BACKWARD motor command. Keep -1 for all four motors unless a
// single wheel is rewired or mounted differently.
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = {-1, -1, -1, -1};

// Encoder signs are defined in the ROS wheel frame, not in the motor driver's
// electrical frame. After reversing all motor outputs above, these signs make
// physical forward motion positive for PID feedback and odometry.
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {1, -1, -1, 1};

// Motor order used throughout this sketch and the ROS URDF:
// 1 front-left, 2 front-right, 3 rear-right, 4 rear-left.
const bool LEFT_SIDE[MOTOR_COUNT] = {true, false, false, true};

// Per-motor feed-forward calibration. Motor 4 is mechanically faster, so its
// starting PWM is reduced. All four motors still receive the same wheel-speed
// target; the independent PID loops then remove the remaining speed error.
// Tune motor 4 in small steps: 0.82 -> 0.80 if still fast, or 0.84 if too slow.
// Direction-specific feed-forward calibration. Some DC motors are not
// symmetrical: the same PWM can produce a different speed in forward and
// reverse. Motor 4 is faster, so it starts with less PWM in both directions.
// Fine tune at runtime with TRIMF,4,<scale> and TRIMR,4,<scale>.
float PWM_FORWARD_SCALE[MOTOR_COUNT] = {1.0f, 1.0f, 1.0f, 0.78f};
float PWM_REVERSE_SCALE[MOTOR_COUNT] = {1.0f, 1.0f, 1.0f, 0.66f};

// Straight-line heading hold. When linear motion is requested with zero yaw
// rate, the current MPU6050 yaw is captured and small left/right wheel target
// corrections are applied to reject motor, floor-friction and load mismatch.
bool straightHoldEnabled = true;
float STRAIGHT_HEADING_KP = 1.8f;       // deg/s correction per degree error
float STRAIGHT_YAW_RATE_KD = 0.12f;     // damping from measured deg/s
float STRAIGHT_MAX_CORRECTION_DEG_S = 24.0f;
const float STRAIGHT_MIN_LINEAR_MM_S = 35.0f;

// MPU sign: a physical left turn must make ROS yaw increase. Set to -1.0 if the
// yaw decreases during a left-turn test.
const float IMU_YAW_SIGN = 1.0f;

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------

QGPMaker_MotorShield motorShield;
QGPMaker_Encoder encoder1(1);
QGPMaker_Encoder encoder2(2);
QGPMaker_Encoder encoder3(3);
QGPMaker_Encoder encoder4(4);
MPU6050 mpu6050(Wire);

// -----------------------------------------------------------------------------
// PID controller
// -----------------------------------------------------------------------------

struct WheelPID {
  float kp;
  float ki;
  float kd;
  float integral;
  float previousError;
  float previousTarget;

  void configure(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
    reset();
  }

  void reset() {
    integral = 0.0f;
    previousError = 0.0f;
    previousTarget = 0.0f;
  }

  float update(float target, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // Clear accumulated integral when stopped or when wheel direction changes.
    if (fabs(target) < 1.0f || (target * previousTarget < 0.0f)) {
      integral = 0.0f;
      previousError = 0.0f;
    }

    const float error = target - measured;
    integral += error * dt;
    integral = constrain(integral, -500.0f, 500.0f);

    const float derivative = (error - previousError) / dt;
    previousError = error;
    previousTarget = target;

    return kp * error + ki * integral + kd * derivative;
  }
};

WheelPID wheelPID[MOTOR_COUNT];

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------

float commandLinearMMs = 0.0f;
float commandAngularDegS = 0.0f;
float rampedLinearMMs = 0.0f;
float rampedAngularDegS = 0.0f;

float wheelTargetMMs[MOTOR_COUNT] = {0};
float wheelMeasuredMMs[MOTOR_COUNT] = {0};
float wheelFilteredMMs[MOTOR_COUNT] = {0};
float wheelPWM[MOTOR_COUNT] = {0};
int32_t cumulativeTicks[MOTOR_COUNT] = {0};

float poseXMM = 0.0f;
float poseYMM = 0.0f;
float yawDeg = 0.0f;
float yawRateDegS = 0.0f;
float yawZeroDeg = 0.0f;
float linearVelocityMMs = 0.0f;

bool emergencyStopLatched = false;
bool watchdogStopped = true;
bool timedBrakeActive = false;
bool straightHeadingActive = false;
float straightHeadingTargetDeg = 0.0f;

uint32_t timedBrakeStartedMs = 0;
uint32_t lastCommandMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;

char commandBuffer[96];
uint8_t commandLength = 0;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

QGPMaker_DCMotor *getMotor(uint8_t index) {
  return motorShield.getMotor(index + 1);
}

int32_t readEncoderAndResetAtomic(uint8_t index) {
  noInterrupts();
  int32_t value;
  switch (index) {
    case 0: value = encoder1.readAndReset(); break;
    case 1: value = encoder2.readAndReset(); break;
    case 2: value = encoder3.readAndReset(); break;
    default: value = encoder4.readAndReset(); break;
  }
  interrupts();
  return value;
}

void resetEncoderDeltas() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    readEncoderAndResetAtomic(i);
    wheelMeasuredMMs[i] = 0.0f;
    wheelFilteredMMs[i] = 0.0f;
  }
}

float normalizeDegrees(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float moveToward(float current, float target, float maximumStep) {
  if (current < target) return min(current + maximumStep, target);
  if (current > target) return max(current - maximumStep, target);
  return current;
}

float speedFeedForwardPWM(float targetMMs, uint8_t motorIndex) {
  const float magnitude = fabs(targetMMs);
  if (magnitude < 1.0f) return 0.0f;

  float ratio = magnitude / MAX_WHEEL_MM_S;
  ratio = constrain(ratio, 0.0f, 1.0f);
  float pwm = MIN_EFFECTIVE_PWM + ratio * (MAX_DRIVE_PWM - MIN_EFFECTIVE_PWM);

  const float scale = (targetMMs >= 0.0f)
    ? PWM_FORWARD_SCALE[motorIndex]
    : PWM_REVERSE_SCALE[motorIndex];

  return pwm * scale;
}

void releaseAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    QGPMaker_DCMotor *m = getMotor(i);
    m->setSpeed(0);
    m->run(RELEASE);
    wheelPWM[i] = 0.0f;
    wheelTargetMMs[i] = 0.0f;
    wheelPID[i].reset();
  }
}

void brakeAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    QGPMaker_DCMotor *m = getMotor(i);
    m->setSpeed(0);
    m->run(BRAKE);
    wheelPWM[i] = 0.0f;
    wheelTargetMMs[i] = 0.0f;
    wheelPID[i].reset();
  }
}

void applyMotorOutput(uint8_t index, float signedPWM) {
  if (fabs(wheelTargetMMs[index]) < 1.0f || fabs(signedPWM) < 1.0f) {
    QGPMaker_DCMotor *m = getMotor(index);
    m->setSpeed(0);
    m->run(RELEASE);
    wheelPWM[index] = 0.0f;
    return;
  }

  uint8_t pwm = (uint8_t)constrain((int)(fabs(signedPWM) + 0.5f), 0, 255);

  // Convert the logical ROS wheel direction into the motor driver's physical
  // electrical direction. This keeps ROS +X forward and +yaw left/CCW while
  // allowing for motors that are mounted opposite to QGPMaker FORWARD.
  const float physicalTarget = wheelTargetMMs[index] * MOTOR_DIRECTION_SIGN[index];
  const uint8_t direction = (physicalTarget >= 0.0f) ? FORWARD : BACKWARD;

  // Run direction first so the library's internal MDIR is initialised before
  // setSpeed() calls run(MDIR).
  QGPMaker_DCMotor *m = getMotor(index);
  m->run(direction);
  m->setSpeed(pwm);

  // Telemetry remains in the logical ROS wheel frame.
  wheelPWM[index] = (wheelTargetMMs[index] >= 0.0f) ? pwm : -((float)pwm);
}

void startTimedBrake() {
  commandLinearMMs = 0.0f;
  commandAngularDegS = 0.0f;
  rampedLinearMMs = 0.0f;
  rampedAngularDegS = 0.0f;
  timedBrakeActive = true;
  timedBrakeStartedMs = millis();
  brakeAllMotors();
}

void setVelocityCommand(float linearMMs, float angularDegS) {
  const float newLinear = constrain(
    linearMMs,
    -MAX_LINEAR_MM_S,
    MAX_LINEAR_MM_S
  );
  const float newAngular = constrain(
    angularDegS,
    -MAX_ANGULAR_DEG_S,
    MAX_ANGULAR_DEG_S
  );

  const bool zeroCommand =
    fabs(newLinear) < 1.0f && fabs(newAngular) < 0.5f;

  const bool wasMoving =
    fabs(commandLinearMMs) >= 1.0f ||
    fabs(commandAngularDegS) >= 0.5f ||
    fabs(rampedLinearMMs) >= 1.0f ||
    fabs(rampedAngularDegS) >= 0.5f;

  lastCommandMs = millis();
  watchdogStopped = false;

  // A transition from motion to VEL,0,0 uses active braking for one second.
  // Repeated zero commands do not restart the timer.
  if (zeroCommand) {
    commandLinearMMs = 0.0f;
    commandAngularDegS = 0.0f;

    if (wasMoving && !timedBrakeActive) {
      startTimedBrake();
    }
    return;
  }

  // A new non-zero command cancels the temporary brake immediately.
  timedBrakeActive = false;
  commandLinearMMs = newLinear;
  commandAngularDegS = newAngular;
}

void stopCommand(bool brake) {
  timedBrakeActive = false;
  commandLinearMMs = 0.0f;
  commandAngularDegS = 0.0f;
  rampedLinearMMs = 0.0f;
  rampedAngularDegS = 0.0f;
  if (brake) brakeAllMotors();
  else releaseAllMotors();
}

void resetOdometry() {
  poseXMM = 0.0f;
  poseYMM = 0.0f;
  linearVelocityMMs = 0.0f;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) cumulativeTicks[i] = 0;
  resetEncoderDeltas();
  yawZeroDeg = IMU_YAW_SIGN * mpu6050.getAngleZ();
  yawDeg = 0.0f;
}

void zeroYaw() {
  yawZeroDeg = IMU_YAW_SIGN * mpu6050.getAngleZ();
  yawDeg = 0.0f;
}

// -----------------------------------------------------------------------------
// Command parser
// -----------------------------------------------------------------------------

void printReady() {
  Serial.println(F("READY,mDetect_ROS2_Arduino"));
}

void processCommand(char *line) {
  char *savePtr = NULL;
  char *command = strtok_r(line, ",", &savePtr);
  if (command == NULL) return;

  // Make command comparison case insensitive.
  for (char *p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = *p - ('a' - 'A');
  }

  if (strcmp(command, "VEL") == 0) {
    char *linearText = strtok_r(NULL, ",", &savePtr);
    char *angularText = strtok_r(NULL, ",", &savePtr);
    if (linearText && angularText && !emergencyStopLatched) {
      setVelocityCommand(atof(linearText), atof(angularText));
      Serial.println(F("ACK,VEL"));
    } else if (emergencyStopLatched) {
      Serial.println(F("ERR,ESTOP_LATCHED"));
    } else {
      Serial.println(F("ERR,VEL_FORMAT"));
    }
  }
  else if (strcmp(command, "FORWARD") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(value ? fabs(atof(value)) : 100.0f, 0.0f);
  }
  else if (strcmp(command, "REVERSE") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(-(value ? fabs(atof(value)) : 100.0f), 0.0f);
  }
  else if (strcmp(command, "LEFT") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(0.0f, value ? fabs(atof(value)) : 45.0f);
  }
  else if (strcmp(command, "RIGHT") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(0.0f, -(value ? fabs(atof(value)) : 45.0f));
  }
  else if (strcmp(command, "STOP") == 0) {
    startTimedBrake();
    lastCommandMs = millis();
    watchdogStopped = false;
    Serial.println(F("ACK,STOP_BRAKE_1S"));
  }
  else if (strcmp(command, "ESTOP") == 0) {
    emergencyStopLatched = true;
    stopCommand(true);
    Serial.println(F("ACK,ESTOP_LATCHED"));
  }
  else if (strcmp(command, "CLEAR_ESTOP") == 0) {
    emergencyStopLatched = false;
    stopCommand(false);
    lastCommandMs = millis();
    watchdogStopped = false;
    Serial.println(F("ACK,ESTOP_CLEARED"));
  }
  else if (strcmp(command, "RESET_ODOM") == 0) {
    stopCommand(false);
    resetOdometry();
    Serial.println(F("ACK,ODOM_RESET"));
  }
  else if (strcmp(command, "ZERO_YAW") == 0) {
    zeroYaw();
    Serial.println(F("ACK,YAW_ZEROED"));
  }
  else if (strcmp(command, "CAL_IMU") == 0) {
    stopCommand(true);
    Serial.println(F("INFO,KEEP_ROBOT_STILL_CALIBRATING_IMU"));
    mpu6050.calcGyroOffsets(false);
    zeroYaw();
    Serial.println(F("ACK,IMU_CALIBRATED"));
  }
  else if (strcmp(command, "PID") == 0) {
    char *pText = strtok_r(NULL, ",", &savePtr);
    char *iText = strtok_r(NULL, ",", &savePtr);
    char *dText = strtok_r(NULL, ",", &savePtr);
    if (pText && iText && dText) {
      const float kp = atof(pText);
      const float ki = atof(iText);
      const float kd = atof(dText);
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) wheelPID[i].configure(kp, ki, kd);
      Serial.println(F("ACK,PID_ALL"));
    } else {
      Serial.println(F("ERR,PID_FORMAT"));
    }
  }
  else if (strcmp(command, "PIDM") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *pText = strtok_r(NULL, ",", &savePtr);
    char *iText = strtok_r(NULL, ",", &savePtr);
    char *dText = strtok_r(NULL, ",", &savePtr);
    int motorIndex = motorText ? atoi(motorText) - 1 : -1;
    if (motorIndex >= 0 && motorIndex < MOTOR_COUNT && pText && iText && dText) {
      wheelPID[motorIndex].configure(atof(pText), atof(iText), atof(dText));
      Serial.println(F("ACK,PID_MOTOR"));
    } else {
      Serial.println(F("ERR,PIDM_FORMAT"));
    }
  }
  else if (strcmp(command, "TRIM") == 0 ||
           strcmp(command, "TRIMF") == 0 ||
           strcmp(command, "TRIMR") == 0) {
    const bool setForward = strcmp(command, "TRIMR") != 0;
    const bool setReverse = strcmp(command, "TRIMF") != 0;

    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *scaleText = strtok_r(NULL, ",", &savePtr);
    int motorIndex = motorText ? atoi(motorText) - 1 : -1;
    const float scale = scaleText ? atof(scaleText) : -1.0f;

    if (motorIndex >= 0 && motorIndex < MOTOR_COUNT &&
        scale >= 0.40f && scale <= 1.20f) {
      if (setForward) PWM_FORWARD_SCALE[motorIndex] = scale;
      if (setReverse) PWM_REVERSE_SCALE[motorIndex] = scale;
      wheelPID[motorIndex].reset();

      Serial.print(F("ACK,"));
      Serial.print(command);
      Serial.print(',');
      Serial.print(motorIndex + 1);
      Serial.print(',');
      Serial.println(scale, 3);
    } else {
      Serial.println(F("ERR,TRIM_FORMAT"));
    }
  }
  else if (strcmp(command, "STRAIGHT_ON") == 0) {
    straightHoldEnabled = true;
    straightHeadingActive = false;
    Serial.println(F("ACK,STRAIGHT_ON"));
  }
  else if (strcmp(command, "STRAIGHT_OFF") == 0) {
    straightHoldEnabled = false;
    straightHeadingActive = false;
    Serial.println(F("ACK,STRAIGHT_OFF"));
  }
  else if (strcmp(command, "STRAIGHT") == 0) {
    char *kpText = strtok_r(NULL, ",", &savePtr);
    char *kdText = strtok_r(NULL, ",", &savePtr);
    char *maxText = strtok_r(NULL, ",", &savePtr);
    if (kpText && kdText && maxText) {
      STRAIGHT_HEADING_KP = constrain(atof(kpText), 0.0f, 10.0f);
      STRAIGHT_YAW_RATE_KD = constrain(atof(kdText), 0.0f, 2.0f);
      STRAIGHT_MAX_CORRECTION_DEG_S = constrain(atof(maxText), 0.0f, 60.0f);
      straightHeadingActive = false;
      Serial.println(F("ACK,STRAIGHT_GAINS"));
    } else {
      Serial.println(F("ERR,STRAIGHT_FORMAT"));
    }
  }
  else if (strcmp(command, "PING") == 0) {
    Serial.println(F("PONG"));
  }
  else {
    Serial.print(F("ERR,UNKNOWN_COMMAND,"));
    Serial.println(command);
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ':') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
      Serial.println(F("ERR,COMMAND_TOO_LONG"));
    }
  }
}

// -----------------------------------------------------------------------------
// Control and odometry
// -----------------------------------------------------------------------------

void updateIMU() {
  mpu6050.update();
  yawDeg = normalizeDegrees(IMU_YAW_SIGN * mpu6050.getAngleZ() - yawZeroDeg);
  yawRateDegS = IMU_YAW_SIGN * mpu6050.getGyroZ();
}

void updateControl(float dt) {
  // Communication loss is always handled locally on the Arduino.
  if (!emergencyStopLatched &&
      !watchdogStopped &&
      millis() - lastCommandMs > COMMAND_WATCHDOG_MS) {
    watchdogStopped = true;
    startTimedBrake();
  }

  // Normal STOP and watchdog stop actively brake for one second. After that,
  // the H-bridge is released so the motors can coast and are not held hot.
  if (timedBrakeActive &&
      millis() - timedBrakeStartedMs >= STOP_BRAKE_DURATION_MS) {
    timedBrakeActive = false;
    releaseAllMotors();
    Serial.println(F("INFO,STOP_BRAKE_RELEASED"));
  }

  if (emergencyStopLatched) {
    brakeAllMotors();
    return;
  }

  rampedLinearMMs = moveToward(
    rampedLinearMMs,
    commandLinearMMs,
    LINEAR_ACCEL_MM_S2 * dt
  );
  rampedAngularDegS = moveToward(
    rampedAngularDegS,
    commandAngularDegS,
    ANGULAR_ACCEL_DEG_S2 * dt
  );

  float effectiveAngularDegS = rampedAngularDegS;
  const bool straightRequested =
    straightHoldEnabled &&
    fabs(commandAngularDegS) < 0.5f &&
    fabs(rampedLinearMMs) >= STRAIGHT_MIN_LINEAR_MM_S;

  if (straightRequested) {
    if (!straightHeadingActive) {
      straightHeadingTargetDeg = yawDeg;
      straightHeadingActive = true;
    }
    const float headingErrorDeg = normalizeDegrees(straightHeadingTargetDeg - yawDeg);
    float headingCorrectionDegS =
      STRAIGHT_HEADING_KP * headingErrorDeg - STRAIGHT_YAW_RATE_KD * yawRateDegS;
    headingCorrectionDegS = constrain(
      headingCorrectionDegS,
      -STRAIGHT_MAX_CORRECTION_DEG_S,
      STRAIGHT_MAX_CORRECTION_DEG_S
    );
    effectiveAngularDegS += headingCorrectionDegS;
  } else {
    straightHeadingActive = false;
  }

  const float angularRadS = effectiveAngularDegS * DEG_TO_RAD;
  const float leftTarget = rampedLinearMMs - angularRadS * TRACK_WIDTH_MM * 0.5f;
  const float rightTarget = rampedLinearMMs + angularRadS * TRACK_WIDTH_MM * 0.5f;

  float leftDistanceMM = 0.0f;
  float rightDistanceMM = 0.0f;
  uint8_t leftCount = 0;
  uint8_t rightCount = 0;

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const int32_t rawTicks = readEncoderAndResetAtomic(i);
    const int32_t signedTicks = rawTicks * ENCODER_SIGN[i];
    cumulativeTicks[i] += signedTicks;

    const float distanceMM = signedTicks * MM_PER_COUNT;
    const float rawSpeedMMs = distanceMM / dt;

    // Low-pass encoder speed while preserving signed direction.
    wheelFilteredMMs[i] += 0.35f * (rawSpeedMMs - wheelFilteredMMs[i]);
    wheelMeasuredMMs[i] = wheelFilteredMMs[i];

    if (LEFT_SIDE[i]) {
      leftDistanceMM += distanceMM;
      ++leftCount;
      wheelTargetMMs[i] = constrain(leftTarget, -MAX_WHEEL_MM_S, MAX_WHEEL_MM_S);
    } else {
      rightDistanceMM += distanceMM;
      ++rightCount;
      wheelTargetMMs[i] = constrain(rightTarget, -MAX_WHEEL_MM_S, MAX_WHEEL_MM_S);
    }

    const float feedForward = speedFeedForwardPWM(wheelTargetMMs[i], i);

    // Control wheel-speed magnitude only. Direction is applied separately in
    // applyMotorOutput(). This prevents an incorrect encoder sign from making
    // the PID increase PWM during reverse when a wheel is already too fast.
    const float targetMagnitude = fabs(wheelTargetMMs[i]);
    const float measuredMagnitude = fabs(wheelMeasuredMMs[i]);
    const float correction = wheelPID[i].update(
      targetMagnitude,
      measuredMagnitude,
      dt
    );

    float outputMagnitude = feedForward + correction;
    outputMagnitude = constrain(outputMagnitude, 0.0f, 255.0f);

    if (timedBrakeActive) {
      QGPMaker_DCMotor *m = getMotor(i);
      m->setSpeed(0);
      m->run(BRAKE);
      wheelPWM[i] = 0.0f;
      wheelTargetMMs[i] = 0.0f;
      wheelPID[i].reset();
    } else {
      applyMotorOutput(
        i,
        (wheelTargetMMs[i] >= 0.0f) ? outputMagnitude : -outputMagnitude
      );
    }
  }

  if (leftCount > 0) leftDistanceMM /= leftCount;
  if (rightCount > 0) rightDistanceMM /= rightCount;
  const float centreDistanceMM = 0.5f * (leftDistanceMM + rightDistanceMM);

  const float yawRad = yawDeg * DEG_TO_RAD;
  poseXMM += centreDistanceMM * cos(yawRad);
  poseYMM += centreDistanceMM * sin(yawRad);
  linearVelocityMMs = centreDistanceMM / dt;
}

void publishTelemetry() {
  Serial.print(F("T,"));
  Serial.print(millis());
  Serial.print(','); Serial.print(poseXMM, 2);
  Serial.print(','); Serial.print(poseYMM, 2);
  Serial.print(','); Serial.print(yawDeg, 3);
  Serial.print(','); Serial.print(linearVelocityMMs, 2);
  Serial.print(','); Serial.print(yawRateDegS, 3);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(cumulativeTicks[i]);
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(wheelMeasuredMMs[i], 2);
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(wheelPWM[i], 1);
  }

  Serial.print(','); Serial.print(emergencyStopLatched ? 1 : 0);
  Serial.print(','); Serial.println(watchdogStopped ? 1 : 0);
}

// -----------------------------------------------------------------------------
// Arduino setup/loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000UL);

  motorShield.begin(500);
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) getMotor(i);

  // Same initial PID gains for all wheels. Each motor receives the same speed
  // target for straight motion, while its own PID output corrects speed mismatch.
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    wheelPID[i].configure(0.25f, 0.034f, 0.003f);
  }

  // Motor 4 needs a stronger correction because it runs faster than the other
  // three motors at the same raw PWM. This PID acts on measured encoder speed.
  wheelPID[3].configure(0.48f, 0.050f, 0.001f);

  mpu6050.begin();
  Serial.println(F("INFO,KEEP_ROBOT_STILL_IMU_STARTUP_CALIBRATION"));
  mpu6050.calcGyroOffsets(false);

  releaseAllMotors();
  resetEncoderDeltas();
  zeroYaw();

  lastCommandMs = millis();
  lastControlMs = millis();
  lastTelemetryMs = millis();
  printReady();
}

void loop() {
  readSerialCommands();
  updateIMU();

  const uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlMs) * 0.001f;
    lastControlMs = now;
    dt = constrain(dt, 0.005f, 0.050f);
    updateControl(dt);
  }

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    publishTelemetry();
  }
}
