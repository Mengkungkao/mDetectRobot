# mDetect ROS 2 TurtleBot3-Style Robot Project

This release separates the robot into a Raspberry Pi hardware layer and an Ubuntu workstation navigation layer, similar to TurtleBot3.

## Fixed hardware mapping

| Device | Default port | Baud rate |
|---|---:|---:|
| COIN-D6 LiDAR | `/dev/ttyUSB0` | 230400 |
| Arduino UNO | `/dev/ttyUSB1` | 500000 |

The setup can create persistent links named `/dev/coin_d6` and `/dev/arduino_mdetect` so the devices do not swap after reboot.

## Main improvements

- The supplied `cspc_lidar` SDK is included in `ros2_ws/src/cspc_lidar` and built on the Raspberry Pi.
- The obsolete SDK lifecycle launch file was replaced with a normal ROS 2 node launch.
- The SDK now publishes a normalised `-pi` to `+pi` `/scan`, suitable for Nav2 and the front safety gate.
- The Arduino forward trim compile defect was corrected.
- Motor 4 has separate forward and reverse feed-forward trims.
- Four independent encoder PID loops remain active.
- MPU6050 straight-line heading hold compensates for remaining drift.
- Verification scripts test installation, USB assignment, Arduino protocol, ROS topics, TF and network discovery.
- Motor tests require an explicit safety confirmation before movement.

## Architecture

```text
Ubuntu workstation
  RViz2 + SLAM Toolbox or AMCL + Nav2
  A* global planning + Regulated Pure Pursuit
  waypoint client and keyboard teleop
             |
             | ROS 2 DDS over LAN, domain 30
             v
Raspberry Pi Ubuntu Server 22.04
  COIN-D6 SDK -> /scan
  command priority and obstacle safety gate
  Arduino serial bridge -> /odom, /imu/data, /joint_states and TF
             |
             | USB serial at 500000 baud
             v
Arduino UNO
  MPU6050 + four encoders + four motor PID loops
  direction trims + heading hold + watchdog + emergency stop
```

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for more detail.

# 1. Upload the Arduino firmware

Open:

```text
arduino/autonomous_v12/autonomous_v12.ino
```

Install `MPU6050_tockn` from Arduino Library Manager, select **Arduino UNO**, and upload. The QGPMaker and pin-change interrupt files are already in the sketch folder.

Keep the robot completely still during power-up gyro calibration.

# 2. Install the Raspberry Pi

Target operating system: **Ubuntu Server 22.04, 64-bit**.

From the extracted project directory:

```bash
chmod +x install_pi.sh
./install_pi.sh
```

Log out and back in once after installation so the `dialout` group is active.

Verify the installation and physical ports:

```bash
bash scripts/pi/verify_install.sh
```

The verification script expects:

```text
/dev/ttyUSB0 = LiDAR
/dev/ttyUSB1 = Arduino UNO
```

To create stable device names manually:

```bash
bash scripts/pi/configure_udev.sh /dev/ttyUSB0 /dev/ttyUSB1
```

# 3. Install the Ubuntu workstation

Target operating system: **Ubuntu 22.04 Desktop**.

```bash
chmod +x install_workstation.sh
./install_workstation.sh
```

Both machines use:

```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
```

The installers save these values in `~/.mdetect_ros2_env`.

# 4. Bring up the complete robot on the Pi

```bash
./bringup_pi.sh
```

This starts:

- `robot_state_publisher`
- `cspc_lidar` on `/dev/ttyUSB0`
- `mdetect_serial_bridge` on `/dev/ttyUSB1`
- `mdetect_cmd_mux`
- LiDAR front-obstacle safety gate
- odometry, IMU, joint state, wheel telemetry and TF publishers

In a second Pi terminal, verify the running robot:

```bash
./verify_pi.sh
```

Expected core topics:

```text
/scan
/odom
/imu/data
/joint_states
/base/wheel_speeds_mm_s
/base/wheel_pwm
/base/encoder_ticks
/safety/front_blocked
/safety/front_distance
```

# 5. Verify workstation-to-Pi ROS networking

On the workstation:

```bash
bash scripts/workstation/verify_network.sh
```

If topics are not visible, confirm both computers are on the same network, use domain ID 30, and allow DDS multicast/UDP traffic through the firewall.

# 6. Start mapping or navigation

For a new map:

```bash
bash scripts/workstation/bringup_slam.sh
```

For a saved map:

```bash
bash scripts/workstation/bringup_navigation.sh /absolute/path/to/map.yaml
```

Keyboard teleoperation:

```bash
bash scripts/workstation/teleop.sh
```

Predefined waypoint client:

```bash
bash scripts/workstation/run_waypoints.sh
```

# 7. Test motors and correct straight-line motion

First lift the robot so all wheels are clear:

```bash
bash scripts/pi/test_actuators_on_blocks.sh --confirm-robot-lifted
```

Then provide at least 2 m of clear floor and run the low-speed report:

```bash
bash scripts/pi/straight_line_test.sh --confirm-clear-path
```

Example motor 4 forward adjustment:

```bash
bash scripts/pi/set_motor_tuning.sh trimf 4 0.80
```

Display current Arduino runtime tuning:

```bash
bash scripts/pi/set_motor_tuning.sh show
```

See [docs/STRAIGHT_LINE_TUNING.md](docs/STRAIGHT_LINE_TUNING.md).

# 8. Emergency controls

```bash
ros2 service call /base/emergency_stop std_srvs/srv/Trigger {}
ros2 service call /base/clear_emergency_stop std_srvs/srv/Trigger {}
ros2 service call /base/reset_odometry std_srvs/srv/Trigger {}
ros2 service call /base/zero_yaw std_srvs/srv/Trigger {}
ros2 service call /base/calibrate_imu std_srvs/srv/Trigger {}
```

# 9. Optional Pi auto-start

After manual bringup and verification work correctly:

```bash
bash scripts/pi/enable_autostart.sh
```

Check the service:

```bash
systemctl status mdetect-robot.service
journalctl -u mdetect-robot.service -f
```

# ROS coordinate convention

- `+X`: forward
- `+Y`: left
- positive yaw: counter-clockwise
- mapping TF: `map -> odom -> base_footprint -> base_link -> laser`

# Important safety behaviour

- Arduino communication watchdog stops the motors after 500 ms without commands.
- Emergency stop remains latched until explicitly cleared.
- Normal stop brakes for one second, then releases the H-bridge.
- The Pi blocks positive forward velocity when an obstacle is closer than 0.30 m in the front cone.
- Rotation remains available while blocked so the robot can turn away.
