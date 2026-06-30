# mDetectRobot

Autonomous differential-drive robot based on a TurtleBot3-style chassis with a custom 4-wheel motor controller, MPU6050 IMU, and CSPC D4 LiDAR. Runs ROS 2 Humble on a Raspberry Pi.

---

## Hardware

| Component | Detail |
|---|---|
| MCU | Arduino Uno |
| Motor shield | QGPMaker 4-channel DC motor shield |
| Encoders | QGPMaker quadrature encoders × 4 |
| IMU | MPU6050 (I²C) |
| LiDAR | CSPC D4 (serial, 230 400 baud) |
| SBC | Raspberry Pi (ROS 2 Humble) |
| Wheel diameter | 80.5 mm |
| Wheel separation | 235 mm |

### Port assignments

| Device | Default port |
|---|---|
| Arduino Uno | `/dev/ttyUSB1` |
| CSPC D4 LiDAR | `/dev/ttyUSB0` |

---

## Repository layout

```
mDetectRobot/
├── arduino/
│   └── turtlebot3_arduino/
│       ├── turtlebot3_arduino.ino   # Arduino firmware
│       ├── QGPMaker_MotorShield.*   # Motor shield driver
│       ├── QGPMaker_Encoder.h       # Quadrature encoder driver
│       └── PinChangeInterrupt*      # Interrupt library
└── turtlebot3_ws/
    └── src/
        ├── arduino_bridge/          # ROS 2 Python package
        │   ├── arduino_bridge/
        │   │   └── arduino_bridge_node.py
        │   ├── launch/
        │   │   ├── robot.launch.py        # Robot bringup (LiDAR + bridge)
        │   │   ├── arduino_bridge.launch.py
        │   │   └── workstation.launch.py  # SLAM + Nav2 + RViz
        │   └── param/
        │       ├── arduino_bridge.yaml
        │       ├── slam_params.yaml
        │       └── nav2_params.yaml
        └── cspc_lidar/              # ROS 2 C++ package
            ├── src/node_lidar_ros.cpp
            ├── sdk/                 # LiDAR SDK
            ├── launch/cspc_lidar.launch.py
            └── params/cspc_lidar.yaml
```

---

## Node graph

```
                        ┌──────────────────────────────┐
teleop / nav2 ─────────►│                              │
  /cmd_vel (Twist)      │     arduino_bridge_node      │◄──► Arduino /dev/ttyUSB1
                        │     (port /dev/ttyUSB1)      │     500 000 baud
                        └──────────────────────────────┘
                          │ /odom  (Odometry)  ──► slam_toolbox, nav2
                          │ /imu   (Imu)       ──► nav2
                          │ /tf    odom→base   ──► TF tree
                          │ /lidar_status (UInt16)
                          ▼
                        ┌──────────────────────────────┐
                        │       cspc_lidar_node        │◄──► LiDAR /dev/ttyUSB0
                        │     (port /dev/ttyUSB0)      │     230 400 baud
                        └──────────────────────────────┘
                          │ /scan  (LaserScan) ──► slam_toolbox, nav2

slam_toolbox ──► /map (OccupancyGrid), map→odom TF
nav2         ──► /cmd_vel
```

---

## Serial protocol (Arduino ↔ Pi)

**Baud rate:** 500 000

### Pi → Arduino

| Command | Format | Description |
|---|---|---|
| Open | `O,seq` | Full state reset; Arduino replies `A,seq,0` then `READY` |
| Velocity | `V,seq,m1,m2,m3,m4` | Wheel targets in mm/s (L1, L2, R1, R2) |
| Brake | `B,seq,hold_ms` | Apply brake for `hold_ms`, then release |
| Stop | `S,seq` | Normal stop: brake 1 s, then release |
| E-stop | `E,seq` | Latch emergency stop |
| Clear | `C,seq` | Clear emergency stop |
| Reset | `R,seq` | Zero encoders and yaw reference |
| PID | `P,seq,motor,kp,ki,kd` | Update PID gains (motor 1–4) |
| Query | `Q,seq` | Request immediate telemetry |
| Close | `X,seq` | Stop motors cleanly, enter RELEASED |

### Arduino → Pi

| Message | Format | Description |
|---|---|---|
| Ready | `READY` | Sent on boot and after every `O` command |
| Telemetry | `T,seq,ms,e1,e2,e3,e4,v1,v2,v3,v4,yaw_cdeg,gyro_cdeg_s,state,fault` | 50 Hz odometry + IMU |
| Ack | `A,seq,code` | 0 = ok, 1 = parse error, 2 = rejected (e-stop latched) |

### Arduino controller states

| Code | Name | Meaning |
|---|---|---|
| 0 | RELEASED | Idle, motors coasting |
| 1 | RUNNING | PID active |
| 2 | BRAKING | Timed brake in progress |
| 3 | ESTOP | Emergency stop latched |
| 4 | WATCHDOG | No command received within 250 ms |
| 5 | FAULT | Hardware fault |

---

## LiDAR control

The bridge node publishes `UInt16` on `/lidar_status` to coordinate the LiDAR with the Arduino connection state:

| Value | Effect on cspc_lidar |
|---|---|
| `1` | Start scanning (sent after Arduino READY) |
| `2` | Stop scanning (sent on Arduino disconnect) |

Set `control_lidar: false` in `arduino_bridge.yaml` to run the LiDAR independently.

---

## Dependencies

**On the Raspberry Pi:**

```bash
sudo apt install \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-turtlebot3-description \
  ros-humble-xacro \
  python3-serial
```

**On the workstation:**

```bash
sudo apt install \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3-navigation2
```

## Building

```bash
cd ~/mDetectRobot/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select arduino_bridge cspc_lidar
source install/setup.bash
```

---

## Flashing the Arduino

Open `arduino/turtlebot3_arduino/turtlebot3_arduino.ino` in the Arduino IDE and upload to the Uno. Required libraries (install via Library Manager):

- `MPU6050_tockn`

The `QGPMaker_MotorShield`, `QGPMaker_Encoder`, and `PinChangeInterrupt` libraries are bundled in the sketch folder.

---

## Running on the robot

**Identify USB ports before launching** — plug in devices one at a time and note which port appears:

```bash
ls /dev/ttyUSB*
# or for more detail:
udevadm info --name=/dev/ttyUSB0 | grep -E "ID_VENDOR|ID_MODEL"
udevadm info --name=/dev/ttyUSB1 | grep -E "ID_VENDOR|ID_MODEL"
```

**Terminal 1 — robot bringup (LiDAR + Arduino bridge):**

```bash
source ~/mDetectRobot/turtlebot3_ws/install/setup.bash
ros2 launch arduino_bridge robot.launch.py \
  lidar_port:=/dev/ttyUSB0 \
  arduino_port:=/dev/ttyUSB1
```

> If the Arduino bridge logs `Serial error: No such file or directory`, it will also print
> all available `/dev/ttyUSB*` and `/dev/ttyACM*` ports — use that output to pass the
> correct `arduino_port:=` argument.

**Terminal 2 — SLAM + Nav2 + RViz (workstation):**

```bash
source ~/mDetectRobot/turtlebot3_ws/install/setup.bash
ros2 launch arduino_bridge workstation.launch.py
```

---

## Tuning

### PID gains

Default gains are set in [turtlebot3_arduino.ino](arduino/turtlebot3_arduino/turtlebot3_arduino.ino) in `setupPID()`. To update gains at runtime without reflashing, send the P command directly over serial:

```
P,seq,motor,kp,ki,kd
```

For example, to set motor 1 gains: `P,1,1,0.25,0.034,0.003`

Open a serial terminal at 500 000 baud on `/dev/ttyUSB1` and type the command, or use the Arduino IDE serial monitor.

### Motor feedforward

`MOTOR_FEEDFORWARD_SCALE` in the `.ino` compensates for per-motor friction differences. Tune by commanding a fixed speed and comparing measured vs target.

### Encoder and motor direction

- `ENCODER_SIGN[i]` — flip to `-1` if a wheel's encoder counts backwards when driving forward.
- `MOTOR_DIRECTION_SIGN[i]` — flip to `-1` if a motor runs the wrong way for a given PWM sign.

---

## Key parameters (`arduino_bridge.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyUSB1` | Arduino serial port |
| `baud_rate` | `500000` | Must match firmware |
| `wheel_separation` | `0.235` | metres, left-to-right contact patch |
| `wheel_radius` | `0.04025` | metres (80.5 mm diameter / 2) |
| `cmd_vel_timeout` | `0.5` | Seconds before sending a stop if `/cmd_vel` goes silent |
| `publish_tf` | `true` | Publish `odom → base_footprint` TF |
| `control_lidar` | `true` | Publish `/lidar_status` on connect/disconnect |
