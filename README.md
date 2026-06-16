# mDetect TurtleBot3-style ROS 2 Humble Architecture

This starter project reorganises the existing mDetect robot into the same general split used by TurtleBot3:

- **Ubuntu workstation:** RViz2, SLAM Toolbox, Nav2, global/local costmaps, Smac 2D A*, Regulated Pure Pursuit, waypoint mission client.
- **Raspberry Pi:** COIN-D6 ROS driver, robot description/TF, Arduino serial bridge, odometry publication, Pi-local Collision Monitor.
- **Arduino Uno:** four encoder loops, four signed wheel-speed targets, individual PWM, MPU6050 yaw, command watchdog, brake hold, and emergency-stop latch.

## 1. Runtime data flow

```text
Waypoint input: (x_mm,y_mm,heading_deg,wait_s)
                    |
                    v
Ubuntu workstation
  waypoint_runner -> Nav2 NavigateToPose
  SLAM Toolbox / AMCL
  global costmap + SmacPlanner2D A*
  local costmap + Rotation Shim + Regulated Pure Pursuit
  velocity smoother
                    |
                    | ROS 2 DDS: /cmd_vel, /scan, /odom, /tf, /map
                    v
Raspberry Pi
  COIN-D6 driver ------------------------------> /scan
  Collision Monitor: /cmd_vel -> /cmd_vel_safe
  serial_bridge: /cmd_vel_safe -> wheel speeds
  serial_bridge: encoder + MPU telemetry -> /odom, /imu/data, TF
                    |
                    | USB serial 500000 baud
                    v
Arduino Uno
  four encoder speed PID loops
  individual motor PWM/direction
  watchdog 250 ms
  1 s brake then release
  hardware/software E-stop latch
```

Normal obstacles enter both costmaps through `/scan`. Nav2 replans with A*. The Pi-local Collision Monitor is the final fast stop layer and does not depend on workstation processing latency.

## 2. Coordinate conversion

The old project convention is preserved at the user interface:

- user `+X` = right
- user `+Y` = forward/up
- heading `0 deg` = `+Y`
- positive heading = clockwise

Nav2 uses ROS REP-103 coordinates:

- ROS `+X` = forward
- ROS `+Y` = left
- positive yaw = counter-clockwise

The waypoint node applies:

```text
ros_x_m = user_y_mm / 1000
ros_y_m = -user_x_mm / 1000
ros_yaw = -heading_deg
```

Therefore:

```text
(0,2000,0,0)       -> ROS (2.0,  0.0, 0 deg)
(1000,2000,0,0)    -> ROS (2.0, -1.0, 0 deg)
(1000,0,90,5)      -> ROS (0.0, -1.0, -90 deg), wait 5 s
```

## 3. Install ROS packages on both computers

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-rotation-shim-controller \
  ros-humble-nav2-collision-monitor \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  python3-serial python3-colcon-common-extensions
```

Use the same ROS settings on the workstation and Pi:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
```

Add those lines to `~/.bashrc` on both machines. Both machines must be on the same network and able to ping each other. DDS automatically transports `/scan`, `/odom`, TF, maps, costmaps and velocity topics between them.

## 4. Build the workspace

Copy `ros2_ws` to both the workstation and Raspberry Pi, then build:

```bash
cd ~/mdetect_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 5. Upload the Arduino firmware

Open:

```text
arduino/mdetect_uno_nav2/mdetect_uno_nav2.ino
```

Keep the supplied QGPMaker, PinChangeInterrupt and PCA9685 files in the same Arduino project/library environment. Install `MPU6050_tockn` if it is not already installed.

The firmware uses:

- USB serial: `500000` baud
- encoders: `4320` counts/revolution
- wheel diameter: `80.5 mm`
- motor side mapping: M1/M4 left, M2/M3 right
- hardware E-stop: `A0`
- watchdog: `250 ms`
- brake hold: `1 s`, then release

The physical E-stop configuration in the sketch assumes a normally-closed switch between A0 and GND. Change `ESTOP_ACTIVE_LEVEL` when using different wiring.

## 6. Raspberry Pi bringup

The COIN-D6 driver must run on the Pi and publish:

```text
Topic: /scan
Type: sensor_msgs/msg/LaserScan
Frame: laser_frame
Serial: typically /dev/ttyUSB1 at 230400 baud
```

Keep using the existing COIN-D6/cspc ROS 2 driver that already works with the sensor. Its serial parsing should remain separate from the Arduino serial bridge.

### Give both USB devices stable names

Because the Arduino and LiDAR can swap between `/dev/ttyUSB0` and `/dev/ttyUSB1`, create udev links before autonomous testing. First inspect each device separately:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -m1 -E 'idVendor|serial'
udevadm info -a -n /dev/ttyUSB1 | grep -m1 -E 'idVendor|serial'
```

Create `/etc/udev/rules.d/99-mdetect-serial.rules` using the actual vendor/product/serial values. The intended links are:

```text
/dev/mdetect_arduino -> Arduino Uno USB serial
/dev/mdetect_lidar   -> COIN-D6 USB serial
```

Then reload and verify:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/mdetect_*
```

Use `/dev/mdetect_arduino` in `robot_bringup.launch.py` and `/dev/mdetect_lidar` in the COIN-D6 launch file.

Start the LiDAR driver first, then:

```bash
source /opt/ros/humble/setup.bash
source ~/mdetect_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch mdetect_robot robot_bringup.launch.py \
  arduino_port:=/dev/mdetect_arduino \
  arduino_baud:=500000
```

Verify Pi topics:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /arduino/connected
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 7. Workstation: build a map and navigate during SLAM

```bash
source /opt/ros/humble/setup.bash
source ~/mdetect_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch mdetect_robot workstation_mapping.launch.py
```

Set RViz Fixed Frame to `map`. Add these displays when required:

- Map: `/map`
- LaserScan: `/scan`
- Odometry: `/odom`
- TF
- Global Costmap: `/global_costmap/costmap`
- Local Costmap: `/local_costmap/costmap`
- Path: `/plan`
- PoseArray: `/mission_waypoints`
- RobotModel

Save a completed map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/mdetect_map
```

## 8. Workstation: navigate on a saved map

```bash
ros2 launch mdetect_robot workstation_navigation.launch.py \
  map:=$HOME/maps/mdetect_map.yaml
```

Use RViz `2D Pose Estimate` once to initialise AMCL, or publish an initial pose through another node.

## 9. Send predefined multiple waypoints

Example requested by the project:

```bash
ros2 run mdetect_robot waypoint_runner --points \
"(0,2000,0,0) (1000,2000,0,0) (1000,0,90,5) (0,0,0,0)"
```

The node publishes the entered poses on `/mission_waypoints` for RViz, then sends one `NavigateToPose` action at a time. This is intentional: Nav2 Humble's standard `WaitAtWaypoint` plugin has one configured pause value for all waypoints, while this mission format needs a different waiting time for each point.

## 10. Manual safety commands

Brake immediately:

```bash
ros2 service call /arduino/brake std_srvs/srv/Trigger '{}'
```

Latch emergency stop:

```bash
ros2 service call /arduino/emergency_stop std_srvs/srv/Trigger '{}'
```

Clear the latch after the physical E-stop is released and velocity is zero:

```bash
ros2 service call /arduino/clear_emergency_stop std_srvs/srv/Trigger '{}'
```

Reset encoder/ROS odometry only while stationary:

```bash
ros2 service call /arduino/reset_odometry std_srvs/srv/Trigger '{}'
```

## 11. Topic ownership

| Topic or TF | Publisher location | Consumer |
|---|---|---|
| `/scan` | Pi COIN-D6 driver | SLAM, AMCL, costmaps, collision monitor, RViz |
| `/odom` | Pi serial bridge | Nav2, SLAM, RViz |
| `odom -> base_footprint` | Pi serial bridge | Nav2 and SLAM |
| robot fixed TF | Pi robot_state_publisher | all ROS nodes |
| `map -> odom` | workstation SLAM Toolbox or AMCL | complete TF tree |
| `/cmd_vel_nav` | workstation controller server | velocity smoother |
| `/cmd_vel` | workstation velocity smoother | Pi collision monitor |
| `/cmd_vel_safe` | Pi collision monitor | Pi serial bridge |
| USB wheel commands | Pi serial bridge | Arduino Uno |

Only one node should publish each transform. SLAM Toolbox or AMCL owns `map -> odom`; the serial bridge owns `odom -> base_footprint`; robot_state_publisher owns fixed robot transforms.

## 12. Required tuning before autonomous testing

1. Confirm M1/M4 are physically the left side and M2/M3 the right side.
2. Lift the robot and test positive wheel commands at low speed.
3. Correct motor direction or encoder sign constants before floor testing.
4. Measure the real track width and update both `track_width_m` and the URDF.
5. Measure the final robot footprint and update both local/global costmaps.
6. Set the exact COIN-D6 position and yaw in `laser_joint`.
7. Tune each motor PID and `MOTOR_SCALE`, especially motor 4.
8. Begin with `desired_linear_vel: 0.12 m/s` and reduce it for confined tunnels.
9. Test the 250 ms command watchdog by disconnecting Wi-Fi and USB data separately.
10. Test the physical E-stop before starting Nav2.

## 13. Important limitation

Regulated Pure Pursuit publishes forward velocity and yaw rate; it does not use the mecanum base's lateral motion. This configuration deliberately treats the four-motor robot as a TurtleBot3-style differential/skid-steer base. It can still reach arbitrary `(x,y,heading)` goals by driving and rotating, which is the most direct path to stable Nav2 operation with the current hardware.
