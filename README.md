# mDetect ROS 2 TurtleBot3-Style Robot Stack

This package rebuilds the working `mdetect_ros2_v1` project into a TurtleBot3-style ROS 2 Humble system with separate onboard bringup and workstation navigation packages.

## Fixed hardware assignment

| Component | Device | Baud rate |
|---|---:|---:|
| COIN-D6 LiDAR | `/dev/ttyUSB0` | 230400 |
| Arduino Uno | `/dev/ttyUSB1` | 500000 |

The installers do **not** calculate or require a SHA256 checksum.

## Architecture

```text
Ubuntu workstation
  RViz2 + SLAM Toolbox + Navigation2
  A* NavFn planner + Regulated Pure Pursuit
  waypoint and keyboard control
                 |
                 | ROS 2 DDS, ROS_DOMAIN_ID=30
                 v
Raspberry Pi / Ubuntu Server 22.04 / ROS 2 Humble
  mdetect_bringup
  robot_state_publisher and TF
  CSPC vendor COIN-D6 SDK -> /scan
  safety command mux -> /cmd_vel
  Arduino serial bridge -> /odom, /imu/data, /joint_states
                 |
                 | /dev/ttyUSB1, 500000 baud
                 v
Arduino Uno
  four encoders + MPU6050
  four independent speed PID loops
  motor 4 forward/reverse trim
  straight-line IMU heading hold
  motor watchdog, emergency stop, one-second stop brake
```

The split follows the TurtleBot3 operating pattern: essential hardware bringup runs on the SBC, while SLAM, navigation and RViz run on the remote Ubuntu computer.

## Project packages

```text
ros2_ws/src/
  cspc_lidar/          supplied COIN-D6/D4 SDK, patched for ROS 2 Humble
  mdetect_base/        Arduino bridge, command priority and safety, initializer
  mdetect_description/ robot URDF and RViz model
  mdetect_bringup/     Raspberry Pi robot.launch.py and hardware parameters
  mdetect_navigation/ workstation SLAM, Nav2, RViz and waypoint routes
```

## 1. Upload the Arduino firmware

Install `MPU6050_tockn` from Arduino Library Manager, then open:

```text
arduino/mdetect_low_level/mdetect_low_level.ino
```

Select **Arduino Uno** and upload. The QGPMaker and pin-change interrupt sources are already kept in the same sketch folder.

Keep the robot still during startup IMU calibration.

## 2. Install on Raspberry Pi

Target: Ubuntu Server 22.04 Jammy, ROS 2 Humble.

```bash
unzip mdetect_ros2_turtlebot3.zip
cd mdetect_ros2_turtlebot3
./install_pi.sh
```

The script:

- safely reuses an existing official ROS apt source;
- removes only conflicting legacy ROS `.list` entries when `ros2.sources` already exists;
- installs ROS Base and build dependencies;
- copies onboard packages to `~/mdetect_ws/src`;
- builds with one worker for Raspberry Pi memory limits;
- adds the user to `dialout`;
- configures `ROS_DOMAIN_ID=30` and `ROS_LOCALHOST_ONLY=0`.

Log out and back in once after the first installation so the `dialout` group is active.

## 3. Verify and bring up the Pi

Connect the hardware in this order:

```text
/dev/ttyUSB0  COIN-D6 LiDAR
/dev/ttyUSB1  Arduino Uno
```

Run:

```bash
./scripts/verify_pi.sh
./scripts/bringup_pi.sh
```

`bringup_pi.sh` performs preflight checks first, then starts:

- robot description and TF;
- COIN-D6 vendor driver;
- Arduino serial bridge;
- odometry, IMU and wheel joints;
- command priority and front obstacle stop;
- startup initialization and `/robot/ready` monitoring.

From a second Pi terminal, verify the live robot:

```bash
./scripts/verify_pi.sh --live
```

Expected core topics:

```text
/scan
/odom
/imu/data
/joint_states
/tf
/cmd_vel
/robot/ready
/diagnostics
```

### Optional motor and encoder self-test

This test is intentionally **not** run automatically because the wheels move. Lift the robot so every wheel is off the ground, then run:

```bash
./scripts/test_actuators_on_blocks.sh
```

It drives forward and reverse through the normal ROS command path and checks the signed encoder speed from all four motors.

## 4. Install on the Ubuntu workstation

Target: Ubuntu Desktop 22.04 Jammy, ROS 2 Humble.

```bash
cd mdetect_ros2_turtlebot3
./install_workstation.sh
```

Make sure the workstation and Pi are on the same network. Then confirm discovery:

```bash
./scripts/verify_workstation.sh --network
```

## 5. Start SLAM and navigation

Keep Pi bringup running first.

For mapping:

```bash
./scripts/bringup_slam.sh
```

For keyboard control during mapping:

```bash
./scripts/teleop.sh
```

Save a map:

```bash
ros2 run nav2_map_server map_saver_cli -f "$HOME/mdetect_map"
```

For navigation with a saved map:

```bash
./scripts/bringup_navigation.sh "$HOME/mdetect_map.yaml"
```

## 6. Straight-line movement correction

The updated Arduino controller fixes the missing `PWM_FORWARD_SCALE` declaration and uses three correction layers:

1. each wheel has an independent encoder speed PID;
2. motor 4 starts with lower feed-forward PWM because it is mechanically faster;
3. the MPU6050 holds the captured heading when `angular.z = 0`.

Default motor 4 scales:

```text
forward = 0.78
reverse = 0.66
```

Change them while ROS bringup is running:

```bash
./scripts/set_motor4_trim.sh 0.76 0.64
```

Or send individual commands:

```bash
ros2 topic pub --once /base/raw_command std_msgs/msg/String "{data: 'TRIMF,4,0.76'}"
ros2 topic pub --once /base/raw_command std_msgs/msg/String "{data: 'TRIMR,4,0.64'}"
```

Use small changes of `0.02`. If motor 4 is still faster, reduce its scale. If it becomes slower, increase it.

Run a controlled straight test:

```bash
./scripts/test_straight_line.sh 0.10 3
```

The robot will move at 0.10 m/s for 3 seconds after confirmation.

Heading hold commands:

```bash
ros2 topic pub --once /base/raw_command std_msgs/msg/String "{data: 'STRAIGHT_ON'}"
ros2 topic pub --once /base/raw_command std_msgs/msg/String "{data: 'STRAIGHT,1.8,0.12,24'}"
ros2 topic pub --once /base/raw_command std_msgs/msg/String "{data: 'STRAIGHT_OFF'}"
```

## 7. Predefined waypoint routes

Edit:

```text
ros2_ws/src/mdetect_navigation/config/waypoints.yaml
```

Each waypoint is:

```text
[x_m, y_m, yaw_deg, wait_s]
```

Run after Navigation2 is active:

```bash
ros2 run mdetect_navigation waypoint_cli
```

## 8. Safety behaviour

- Arduino communication watchdog stops the motors if Pi commands stop arriving.
- A normal stop actively brakes for one second and then releases the H-bridge.
- Emergency stop remains latched until cleared.
- Forward motion is blocked when an obstacle is inside the configured front sector.
- Forward motion is also blocked if LiDAR data becomes stale.

Emergency stop:

```bash
ros2 service call /base/emergency_stop std_srvs/srv/Trigger '{}'
```

Clear emergency stop:

```bash
ros2 service call /base/clear_emergency_stop std_srvs/srv/Trigger '{}'
```

## 9. Optional automatic startup

After manual bringup works correctly:

```bash
./scripts/install_robot_service.sh
sudo systemctl start mdetect-robot
sudo systemctl status mdetect-robot
```

## References

1. ROBOTIS, “TurtleBot3 Bringup,” TurtleBot3 e-Manual.
2. ROBOTIS, “TurtleBot3 SLAM and Navigation,” TurtleBot3 e-Manual.
3. Open Robotics, “ROS 2 Humble Ubuntu Installation” and “Using colcon to build packages.”
