/*******************************************************************************
 * turtlebot3_style_core.ino
 *
 * A TurtleBot3 / OpenCR-style "core" firmware, rebuilt for an Arduino Uno/Nano
 * with the QGPMaker motor shield + QGPMaker quadrature encoders.
 *
 * It keeps the same skeleton ROBOTIS uses in turtlebot3_core.ino:
 *   - a fixed-rate control loop driven by a small software-timer table (tTime[])
 *   - read encoders  -> calcOdometry() -> integrate (x, y, theta)   [dead reckoning]
 *   - updateGoalVelocity() decides where the robot should go
 *   - controlMotor() closes the loop on each wheel
 *
 * What is DIFFERENT from OpenCR (and why):
 *   - OpenCR drives DYNAMIXEL servos that run their own internal velocity PID.
 *     Here we have plain DC motors over PWM, so we add a per-wheel PID ourselves.
 *   - OpenCR is commanded by ROS /cmd_vel (continuous velocity). Your robot_v12.py
 *     instead streams *waypoints* "(X_mm,Y_mm,wait_s)" and expects pose back as
 *     "X>Y>Heading". So updateGoalVelocity() is a go-to-goal planner over a
 *     waypoint queue rather than a cmd_vel callback.
 *
 * SERIAL PROTOCOL (must match robot_v12.py):
 *   Baud           : 500000
 *   Robot <- Host  : "(X1,Y1,W1) (X2,Y2,W2) ... :\n"   route, X/Y in mm, W in s
 *                    "STOP:\n"                          halt now, clear the queue
 *                    "RESET:\n"                         zero odometry + clear queue
 *   Robot -> Host  : "X>Y>Heading\n"   X,Y in mm (1 dp), Heading in deg (2 dp)
 *
 * >>> CALIBRATE THESE THREE before expecting accurate odometry <<<
 *   WHEEL_DIAMETER_MM, WHEEL_SEPARATION_MM, ENCODER_CPR
 *******************************************************************************/

#include <Wire.h>
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"

/* ============================================================================
 *  CONFIG  (the equivalent of turtlebot3_core_config.h + turtlebot3_burger.h)
 * ========================================================================== */

// ---- Loop rates (Hz). OpenCR uses 30 Hz motor control, 30 Hz drive info. ----
#define CONTROL_FREQUENCY        30      // navigation + wheel PID
#define POSE_PUBLISH_FREQUENCY   20      // how often we send "X>Y>Heading"

// ---- Wheel indices, same naming convention as OpenCR ----
#define LEFT   0
#define RIGHT  1

// ---- Robot geometry  (MEASURE THESE on your own chassis) ----
#define WHEEL_DIAMETER_MM     65.0f      // wheel outer diameter
#define WHEEL_SEPARATION_MM   170.0f     // distance between the two wheel contact points
#define ENCODER_CPR           4320       // encoder counts per WHEEL revolution (x4 decoded)

// ---- Motor shield wiring: which M port is which wheel ----
#define MOTOR_LEFT_PORT   1              // M1
#define MOTOR_RIGHT_PORT  2              // M2

// ---- Direction fixups. Flip a +1 to -1 instead of rewiring/swapping leads. ----
// A wheel is "correct" when driving it forward makes its encoder count UP.
#define MOTOR_LEFT_SIGN    (+1)
#define MOTOR_RIGHT_SIGN   (+1)
#define ENC_LEFT_SIGN      (+1)
#define ENC_RIGHT_SIGN     (+1)

// ---- Speed / approach limits ----
#define MAX_LIN_SPEED_MM_S    250.0f     // forward speed cap
#define MAX_ANG_SPEED_RAD_S   2.5f       // turn-rate cap
#define POSITION_TOLERANCE_MM 25.0f      // "close enough" radius for a waypoint
#define HEADING_GATE_DEG      30.0f      // if heading error is bigger, turn in place first

// ---- Go-to-goal gains ----
#define KP_LINEAR   1.2f                 // mm/s of forward speed per mm of distance error
#define KP_ANGULAR  3.0f                 // rad/s of turn per rad of heading error

// ---- Per-wheel speed PID (target & measurement in mm/s, output in PWM counts) ----
#define MAX_WHEEL_SPEED_MM_S  400.0f     // reference for the feed-forward term
#define WHEEL_KFF   (255.0f / MAX_WHEEL_SPEED_MM_S)   // open-loop guess
#define WHEEL_KP    0.6f
#define WHEEL_KI    0.0f                 // start at 0; add a little once Kp tracks
#define WHEEL_KD    0.0f
#define MOTOR_MIN_PWM   30               // beats static friction so small commands still move
#define MOTOR_MAX_PWM   255

// ---- Derived constants ----
const float WHEEL_CIRCUM_MM = (float)PI * WHEEL_DIAMETER_MM;
const float MM_PER_TICK     = WHEEL_CIRCUM_MM / (float)ENCODER_CPR;   // linear mm per encoder count
const float TICK2RAD        = (2.0f * (float)PI) / (float)ENCODER_CPR; // OpenCR's TICK2RAD analogue
const float HEADING_GATE_RAD = HEADING_GATE_DEG * 0.01745329252f;

// ---- Waypoint queue ----
#define MAX_WAYPOINTS  12                // robot_v12.py caps routes at 10

/* ============================================================================
 *  GLOBALS
 * ========================================================================== */

// Motors + encoders. Encoder "1" = pins D8/D9, "2" = pins D6/D7 (fixed by the library).
QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();
QGPMaker_DCMotor *motorLeft;
QGPMaker_DCMotor *motorRight;
QGPMaker_Encoder  encLeft (1, ENCODER_CPR);   // D8, D9
QGPMaker_Encoder  encRight(2, ENCODER_CPR);   // D6, D7

// Pose (the thing we publish). Internal heading is radians; we output degrees.
float pose_x  = 0.0f;   // mm
float pose_y  = 0.0f;   // mm
float pose_th = 0.0f;   // rad

// Goal wheel speeds set by the planner, consumed by the wheel PID (mm/s).
float goal_wheel[2] = {0.0f, 0.0f};

// Per-wheel PID memory.
float pid_integral[2] = {0.0f, 0.0f};
float pid_prev_err[2] = {0.0f, 0.0f};

// Waypoint queue + navigation state machine.
long  wp_x[MAX_WAYPOINTS];
long  wp_y[MAX_WAYPOINTS];
long  wp_wait[MAX_WAYPOINTS];
int   wp_count = 0;
int   wp_index = 0;

enum NavState { NAV_IDLE, NAV_DRIVE, NAV_WAIT };
NavState nav_state = NAV_IDLE;
unsigned long wait_started_ms = 0;

bool motion_enabled = false;            // false after STOP, true after a fresh route

// Software timers, exactly like OpenCR's tTime[].
uint32_t tTime[2];

// Serial line assembly.
char rxbuf[200];
int  rxlen = 0;

/* ============================================================================
 *  SETUP
 * ========================================================================== */
void setup() {
  Serial.begin(500000);

  AFMS.begin();                          // brings up I2C + the PCA9685 PWM driver
  motorLeft  = AFMS.getMotor(MOTOR_LEFT_PORT);
  motorRight = AFMS.getMotor(MOTOR_RIGHT_PORT);
  stopMotors();

  initOdom();

  uint32_t now = millis();
  tTime[0] = now;
  tTime[1] = now;

  Serial.println(F("READY"));            // ignored by the Python parser, handy on the monitor
}

/* ============================================================================
 *  MAIN LOOP  (mirror of turtlebot3_core.ino's timed blocks)
 * ========================================================================== */
void loop() {
  uint32_t t = millis();

  // Always service incoming commands as fast as possible.
  readSerial();

  // Block 1: navigation + odometry + wheel control at CONTROL_FREQUENCY.
  if ((t - tTime[0]) >= (1000UL / CONTROL_FREQUENCY)) {
    float dt = (t - tTime[0]) / 1000.0f;
    if (dt <= 0.0f) dt = 1.0f / CONTROL_FREQUENCY;

    long dL, dR;
    readEncoders(dL, dR);                // ticks since last control step (atomic)

    float vL_meas = (dL * MM_PER_TICK) / dt;   // measured wheel speed, mm/s
    float vR_meas = (dR * MM_PER_TICK) / dt;

    calcOdometry(dL, dR);                // update pose_x / pose_y / pose_th
    updateGoalVelocity();                // go-to-goal -> goal_wheel[LEFT/RIGHT]
    controlMotor(vL_meas, vR_meas, dt);  // per-wheel PID -> PWM

    tTime[0] = t;
  }

  // Block 2: publish pose "X>Y>Heading" at POSE_PUBLISH_FREQUENCY.
  if ((t - tTime[1]) >= (1000UL / POSE_PUBLISH_FREQUENCY)) {
    publishPose();
    tTime[1] = t;
  }
}

/* ============================================================================
 *  ENCODERS
 * ========================================================================== */
// Read-and-reset both encoders atomically so an interrupt can't land mid-read.
void readEncoders(long &dL, long &dR) {
  noInterrupts();
  dL = encLeft.readAndReset();
  dR = encRight.readAndReset();
  interrupts();
  dL *= ENC_LEFT_SIGN;
  dR *= ENC_RIGHT_SIGN;
}

/* ============================================================================
 *  ODOMETRY  (same midpoint integration as OpenCR's calcOdometry, but in mm)
 * ========================================================================== */
void initOdom() {
  pose_x = pose_y = pose_th = 0.0f;
  long junkL, junkR;
  readEncoders(junkL, junkR);            // clear any startup counts
}

void calcOdometry(long dL, long dR) {
  // OpenCR does: wheel_l = TICK2RAD * dL;  delta_s = R*(wheel_r+wheel_l)/2; etc.
  // With MM_PER_TICK = R * TICK2RAD, that collapses to the lines below.
  float dl_mm = dL * MM_PER_TICK;        // left wheel arc length  (mm)
  float dr_mm = dR * MM_PER_TICK;        // right wheel arc length (mm)

  float delta_s  = 0.5f * (dl_mm + dr_mm);                  // robot forward travel (mm)
  float delta_th = (dr_mm - dl_mm) / WHEEL_SEPARATION_MM;   // heading change (rad)

  // Integrate about the midpoint heading for second-order accuracy.
  pose_x += delta_s * cos(pose_th + 0.5f * delta_th);
  pose_y += delta_s * sin(pose_th + 0.5f * delta_th);
  pose_th = normalizeAngle(pose_th + delta_th);
}

/* ============================================================================
 *  GO-TO-GOAL PLANNER  (stands in for OpenCR's updateGoalVelocity/cmd_vel)
 * ========================================================================== */
void updateGoalVelocity() {
  // Nothing to do, or told to stop: hold still.
  if (!motion_enabled || nav_state == NAV_IDLE || wp_count == 0) {
    goal_wheel[LEFT]  = 0.0f;
    goal_wheel[RIGHT] = 0.0f;
    return;
  }

  // Dwell at a waypoint for its requested "wait" seconds.
  if (nav_state == NAV_WAIT) {
    goal_wheel[LEFT]  = 0.0f;
    goal_wheel[RIGHT] = 0.0f;
    unsigned long wait_ms = (unsigned long)wp_wait[wp_index] * 1000UL;
    if (millis() - wait_started_ms >= wait_ms) advanceWaypoint();
    return;
  }

  // NAV_DRIVE: steer toward the active waypoint.
  float gx = (float)wp_x[wp_index];
  float gy = (float)wp_y[wp_index];
  float dx = gx - pose_x;
  float dy = gy - pose_y;
  float dist = sqrt(dx * dx + dy * dy);

  if (dist <= POSITION_TOLERANCE_MM) {   // arrived
    if (wp_wait[wp_index] > 0) {
      nav_state = NAV_WAIT;
      wait_started_ms = millis();
    } else {
      advanceWaypoint();
    }
    goal_wheel[LEFT]  = 0.0f;
    goal_wheel[RIGHT] = 0.0f;
    return;
  }

  float target_th = atan2(dy, dx);
  float err_th    = normalizeAngle(target_th - pose_th);

  // Forward speed: scale with distance, but back off (and gate) when pointed wrong.
  float v;
  if (fabs(err_th) > HEADING_GATE_RAD) {
    v = 0.0f;                            // turn in place until roughly aligned
  } else {
    v = KP_LINEAR * dist;
    if (v > MAX_LIN_SPEED_MM_S) v = MAX_LIN_SPEED_MM_S;
    v *= cos(err_th);                    // gentle taper as alignment improves
  }

  // Turn rate: proportional to heading error, clamped.
  float w = KP_ANGULAR * err_th;
  if (w >  MAX_ANG_SPEED_RAD_S) w =  MAX_ANG_SPEED_RAD_S;
  if (w < -MAX_ANG_SPEED_RAD_S) w = -MAX_ANG_SPEED_RAD_S;

  // Differential-drive mixing: wheel speeds in mm/s.
  float half_track = WHEEL_SEPARATION_MM * 0.5f;
  goal_wheel[LEFT]  = v - w * half_track;
  goal_wheel[RIGHT] = v + w * half_track;
}

void advanceWaypoint() {
  wp_index++;
  if (wp_index >= wp_count) {            // whole route finished
    nav_state = NAV_IDLE;
    wp_count = 0;
    wp_index = 0;
    motion_enabled = false;
  } else {
    nav_state = NAV_DRIVE;
  }
}

/* ============================================================================
 *  WHEEL CONTROL  (the DC-motor PID that OpenCR delegates to DYNAMIXEL)
 * ========================================================================== */
void controlMotor(float vL_meas, float vR_meas, float dt) {
  if (!motion_enabled || nav_state == NAV_IDLE) {
    stopMotors();
    pid_integral[LEFT] = pid_integral[RIGHT] = 0.0f;
    return;
  }
  int pwmL = wheelPID(LEFT,  goal_wheel[LEFT],  vL_meas, dt);
  int pwmR = wheelPID(RIGHT, goal_wheel[RIGHT], vR_meas, dt);
  setMotor(motorLeft,  MOTOR_LEFT_SIGN,  pwmL);
  setMotor(motorRight, MOTOR_RIGHT_SIGN, pwmR);
}

int wheelPID(int side, float target, float measured, float dt) {
  if (fabs(target) < 1.0f) {             // commanded stop -> release, no creep
    pid_integral[side] = 0.0f;
    pid_prev_err[side] = 0.0f;
    return 0;
  }

  float err = target - measured;
  pid_integral[side] += err * dt;
  // Anti-windup: keep the integral term from exploding.
  float i_clamp = (WHEEL_KI > 0.0f) ? (MOTOR_MAX_PWM / WHEEL_KI) : 0.0f;
  if (i_clamp > 0.0f) {
    if (pid_integral[side] >  i_clamp) pid_integral[side] =  i_clamp;
    if (pid_integral[side] < -i_clamp) pid_integral[side] = -i_clamp;
  }
  float deriv = (err - pid_prev_err[side]) / dt;
  pid_prev_err[side] = err;

  float out = WHEEL_KFF * target          // feed-forward (does most of the work)
            + WHEEL_KP  * err
            + WHEEL_KI  * pid_integral[side]
            + WHEEL_KD  * deriv;

  // Clamp and apply a minimum to overcome stiction.
  int pwm = (int)out;
  if (pwm >  MOTOR_MAX_PWM) pwm =  MOTOR_MAX_PWM;
  if (pwm < -MOTOR_MAX_PWM) pwm = -MOTOR_MAX_PWM;
  if (pwm > 0 && pwm < MOTOR_MIN_PWM) pwm = MOTOR_MIN_PWM;
  if (pwm < 0 && pwm > -MOTOR_MIN_PWM) pwm = -MOTOR_MIN_PWM;
  return pwm;
}

// Drive one motor with a signed PWM value in [-255, 255].
void setMotor(QGPMaker_DCMotor *m, int sign, int pwm) {
  pwm *= sign;
  if (pwm > 0) {
    m->run(FORWARD);
    m->setSpeed(pwm > MOTOR_MAX_PWM ? MOTOR_MAX_PWM : pwm);
  } else if (pwm < 0) {
    m->run(BACKWARD);
    int s = -pwm;
    m->setSpeed(s > MOTOR_MAX_PWM ? MOTOR_MAX_PWM : s);
  } else {
    m->setSpeed(0);
    m->run(RELEASE);
  }
}

void stopMotors() {
  motorLeft->setSpeed(0);
  motorRight->setSpeed(0);
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

/* ============================================================================
 *  POSE OUTPUT  -> "X>Y>Heading\n"  (what read_arduino_lines() parses)
 * ========================================================================== */
void publishPose() {
  float heading_deg = pose_th * 57.2957795131f;   // RAD2DEG
  Serial.print(pose_x, 1);
  Serial.print('>');
  Serial.print(pose_y, 1);
  Serial.print('>');
  Serial.println(heading_deg, 2);
}

/* ============================================================================
 *  SERIAL COMMAND PARSING
 * ========================================================================== */
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxlen > 0) {
        rxbuf[rxlen] = '\0';
        processLine(rxbuf);
        rxlen = 0;
      }
    } else if (rxlen < (int)sizeof(rxbuf) - 1) {
      rxbuf[rxlen++] = c;
    } else {
      rxlen = 0;                         // overflow guard: drop the garbled line
    }
  }
}

void processLine(char *line) {
  if (strncmp(line, "STOP", 4) == 0)  { emergencyStop(); return; }
  if (strncmp(line, "RESET", 5) == 0) { resetAll();      return; }
  if (strchr(line, '(') != NULL)      { parseRoute(line); return; }
  // anything else: ignore
}

// Parse "(x,y,w) (x,y,w) ... :" into the waypoint queue and start driving.
void parseRoute(char *s) {
  wp_count = 0;
  wp_index = 0;

  char *p = s;
  while ((p = strchr(p, '(')) != NULL) {
    long x = 0, y = 0, w = 0;
    if (sscanf(p, "(%ld,%ld,%ld", &x, &y, &w) >= 2) {
      if (wp_count < MAX_WAYPOINTS) {
        wp_x[wp_count]    = x;
        wp_y[wp_count]    = y;
        wp_wait[wp_count] = (w < 0) ? 0 : w;
        wp_count++;
      }
    }
    char *close = strchr(p, ')');
    p = (close != NULL) ? close + 1 : p + 1;
  }

  if (wp_count > 0) {
    motion_enabled = true;
    nav_state = NAV_DRIVE;
  } else {
    motion_enabled = false;
    nav_state = NAV_IDLE;
  }
}

// "STOP:" -> halt immediately and drop the route. Stays stopped until a new route.
void emergencyStop() {
  motion_enabled = false;
  nav_state = NAV_IDLE;
  wp_count = 0;
  wp_index = 0;
  goal_wheel[LEFT] = goal_wheel[RIGHT] = 0.0f;
  pid_integral[LEFT] = pid_integral[RIGHT] = 0.0f;
  stopMotors();
}

// "RESET:" -> zero the odometry frame and clear everything.
void resetAll() {
  emergencyStop();
  initOdom();
}

/* ============================================================================
 *  UTILITY
 * ========================================================================== */
// Wrap an angle to (-PI, PI].
float normalizeAngle(float a) {
  while (a >   (float)PI) a -= 2.0f * (float)PI;
  while (a <= -(float)PI) a += 2.0f * (float)PI;
  return a;
}
