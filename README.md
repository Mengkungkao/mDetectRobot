# AutonomousV11 complete ROS 2 + Arduino starter system

This bundle converts the robot into a three-layer autonomous system:

1. **Ubuntu workstation:** RViz2, map/costmap/path monitoring, SSH, and timed waypoint input.
2. **Raspberry Pi with Ubuntu Server and ROS 2 Humble:** LiDAR `/scan`, TF, encoder/IMU fusion, SLAM Toolbox, Nav2, A* planning, Regulated Pure Pursuit, safety supervision, and the Arduino serial bridge.
3. **Arduino Uno:** four encoders, MPU6050, four wheel-speed PID loops, motor PWM/direction, watchdog, brake/release, and an emergency-stop latch.

The old Arduino waypoint state machine has been removed. Nav2 is now the only global navigation controller.

## Important hardware assumptions

- Wheel diameter: **80.5 mm**
- Encoder resolution: **4320 counts/revolution**
- Track width: **190 mm**
- Motors **1 and 4 are the right side**
- Motors **2 and 3 are the left side**
- Arduino serial speed: **500000 baud**
- LiDAR publishes `sensor_msgs/LaserScan` on `/scan` with frame `lidar_link`
- COIN-D6 serial defaults: `/dev/sc_mini` at **230400 baud**

Change these values in:

- `arduino/AutonomousV11_LowLevel/AutonomousV11_LowLevel.ino`
- `ros2_ws/src/mdetect_base/config/base.yaml`
- `ros2_ws/src/mdetect_description/urdf/mdetect_robot.urdf.xacro`

## 1. Upload the Arduino firmware

Copy these folders into the Arduino library directory:

```bash
cp -r arduino/libraries/QGPMakerRobot ~/Arduino/libraries/
cp -r arduino/libraries/PinChangeInterrupt ~/Arduino/libraries/
```

Install **MPU6050_tockn** from Arduino IDE Library Manager. Open and upload:

```text
arduino/AutonomousV11_LowLevel/AutonomousV11_LowLevel.ino
```

Open Serial Monitor at `500000` baud. After gyro calibration, the Uno prints:

```text
READY
```

### Direct Arduino test commands

Raise the robot so its wheels are clear of the floor.

```text
V,1,50,50,50,50
B,2,1000
V,3,-50,-50,-50,-50
E,4
C,5
R,6
```

The firmware continuously returns encoder counts, wheel speeds, yaw and state using the format documented in `docs/SERIAL_PROTOCOL.md`.

### Direction corrections

If one motor turns the wrong way, change its entry in:

```cpp
const int8_t MOTOR_DIRECTION_SIGN[4] = {1, 1, 1, 1};
```

If one encoder decreases while its wheel moves forward, reverse that entry in:

```cpp
const int8_t ENCODER_SIGN[4] = {-1, 1, 1, -1};
```

If RViz yaw turns clockwise when the robot physically turns counter-clockwise, change:

```cpp
const float IMU_YAW_SIGN = -1.0f;
```

## 2. Install the Raspberry Pi workspace and COIN-D6 driver

Copy this whole folder to the Pi, then run:

```bash
cd AutonomousV11_complete
./scripts/install_pi.sh
```

The installer now:

- installs the PCL and ROS dependencies needed by the CSPC driver;
- copies the bundled `cspc_lidar` package into `~/mdetect_ws/src`;
- installs the COIN-D6 USB rule as `/etc/udev/rules.d/99-cspc-coin-d6.rules`;
- builds the LiDAR driver together with the mDetect packages; and
- configures the full robot launch files to start the LiDAR automatically.

Log out and back in after installation so membership in the `dialout` group takes effect. Unplug and reconnect the LiDAR, then check the device link:

```bash
ls -l /dev/sc_mini
readlink -f /dev/sc_mini
```

Find stable serial paths for both the Arduino and LiDAR:

```bash
ls -l /dev/serial/by-id/
```

Set the Arduino path in:

```bash
nano ~/mdetect_ws/src/mdetect_base/config/base.yaml
```

The default COIN-D6 settings are stored in:

```bash
nano ~/mdetect_ws/src/cspc_lidar/params/cspc_lidar.yaml
```

Default LiDAR values:

```yaml
port: /dev/sc_mini
baudrate: 230400
frame_id: lidar_link
version: 4
```

If you change either configuration file, rebuild:

```bash
cd ~/mdetect_ws
colcon build --symlink-install
source install/setup.bash
```

> The supplied vendor archive is named `cspc_lidar_sdk_ros2_D4_20250731`. It is bundled here as the CSPC driver for the project COIN-D6 unit, but the final hardware behaviour must still be verified on the actual sensor.

### Installer stops at `AMENT_TRACE_SETUP_FILES: unbound variable`

Older copies of the installer used `set -u` while sourcing the ROS 2 Humble environment. Some ament setup scripts read the optional `AMENT_TRACE_SETUP_FILES` variable before defining it, which causes Bash to stop. The current scripts temporarily disable nounset only while sourcing ROS setup files.

For an older package, the immediate terminal workaround is:

```bash
cd ~/mDetectRobot
sed -i '/^source \/opt\/ros\/humble\/setup.bash$/i set +u' scripts/install_pi.sh
sed -i '/^source \/opt\/ros\/humble\/setup.bash$/a set -u' scripts/install_pi.sh
./scripts/install_pi.sh
```

It is safe to rerun the installer. Existing apt packages, copied source files and udev rules are reused.

## 3. Test the COIN-D6 LiDAR

Run the supplied test launcher:

```bash
cd AutonomousV11_complete
./scripts/test_lidar.sh
```

Or start the driver directly:

```bash
source ~/mdetect_ws/install/setup.bash
ros2 launch cspc_lidar lidar_launch.py
```

The expected output is:

```text
Topic: /scan
Type:  sensor_msgs/msg/LaserScan
Frame: lidar_link
Rate:  approximately 10 Hz
```

Check the stream in another terminal:

```bash
ros2 pkg executables cspc_lidar
ros2 topic info /scan
ros2 topic echo /scan --once
ros2 topic hz /scan
```

The driver also publishes `/point_cloud` and diagnostic text on `/lsd_error`.

If `/dev/sc_mini` points to the Arduino instead of the LiDAR, both devices probably use the same USB-to-serial chipset. Use the LiDAR path from `/dev/serial/by-id/` and change the `port` value in `cspc_lidar.yaml`.

See `docs/COIN_D6_LIDAR.md` for detailed setup and troubleshooting.

## 4. Run SLAM + Nav2 on the Pi

The COIN-D6 starts automatically with the full robot launch:

```bash
source ~/mdetect_ws/install/setup.bash
ros2 launch mdetect_bringup robot_slam.launch.py
```

To run the stack without starting the LiDAR, for example when replaying a rosbag:

```bash
ros2 launch mdetect_bringup robot_slam.launch.py start_lidar:=false
```

This starts:

- Arduino serial bridge
- Four-wheel encoder odometry
- MPU6050 IMU publication
- `robot_localization` EKF
- Robot description and TF
- SLAM Toolbox
- Nav2 global and local costmaps
- SmacPlanner2D cost-aware A*
- Regulated Pure Pursuit
- Nav2 velocity smoother
- Front-obstacle safety supervisor
- Timed waypoint executor

## 5. Install and run the Ubuntu workstation

Copy this folder to the workstation and run:

```bash
cd AutonomousV11_complete
./scripts/install_workstation.sh
```

Both computers must use the same network settings:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Open RViz2 on the workstation:

```bash
source ~/mdetect_ws/install/setup.bash
ros2 launch mdetect_bringup workstation_rviz.launch.py
```

Verify that the workstation can see the Pi:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /odometry/filtered --once
```

If discovery does not work, test multicast. Run this on one computer:

```bash
ros2 multicast receive
```

Then run this on the other computer:

```bash
ros2 multicast send
```

Both computers must be on the same LAN, use `ROS_DOMAIN_ID=42`, and allow DDS multicast traffic.

SSH remains useful for Pi maintenance:

```bash
ssh <pi-user>@<pi-ip-address>
```

## 6. Send predefined timed waypoints

The default input mode preserves your original convention:

```text
(x_right_mm, y_forward_mm, clockwise_heading_deg, wait_seconds)
```

Example:

```bash
ros2 run mdetect_base send_waypoints \
  "(0,2000,0,0) (1000,2000,0,0) (1000,0,180,5) (0,0,0,0)"
```

Meaning:

- Move forward 2000 mm
- Move to the point 1000 mm right and 2000 mm forward
- Move to 1000 mm right, face 180°, and wait five seconds
- Return to the origin

The executor converts these values into ROS coordinates and sends each pose to Nav2. It waits for the requested time only after Nav2 reports success.

You can also publish directly:

```bash
ros2 topic pub --once /waypoint_input std_msgs/msg/String \
  "{data: '(0,2000,0,0) (1000,2000,0,5)'}"
```

Cancel the list:

```bash
ros2 service call /waypoints/cancel std_srvs/srv/Trigger "{}"
```

## 7. Safety commands

Latch an emergency stop:

```bash
ros2 service call /safety/stop std_srvs/srv/Trigger "{}"
```

After the obstacle is removed and the front range is clear, reset it:

```bash
ros2 service call /safety/clear std_srvs/srv/Trigger "{}"
```

The system has three independent stop layers:

1. Nav2 collision checking and costmaps
2. Pi front-cone safety supervisor
3. Arduino serial watchdog and emergency-stop latch

A physical power-cut emergency-stop switch is still recommended for testing around people.

## 8. First motor test through ROS 2

Keep the robot raised:

```bash
source ~/mdetect_ws/install/setup.bash
ros2 launch mdetect_bringup robot_bringup.launch.py
```

This launch now starts the COIN-D6 by default. For a motor-only bench test, use:

```bash
ros2 launch mdetect_bringup robot_bringup.launch.py start_lidar:=false
```

In a second terminal:

```bash
./scripts/test_forward.sh 2 0.05
```

Check:

```bash
ros2 topic echo /wheel/encoder_counts
ros2 topic echo /wheel/speeds
ros2 topic echo /imu/data
ros2 topic echo /wheel/odometry
ros2 topic echo /odometry/filtered
```

If the whole robot moves backward for positive `linear.x`, set `linear_sign: -1.0` in `base.yaml`. If positive `angular.z` turns clockwise, set `angular_sign: -1.0`.

## 9. Save a map

After completing mapping:

```bash
mkdir -p ~/mdetect_ws/src/mdetect_bringup/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/mdetect_ws/src/mdetect_bringup/maps/tunnel_map
```

## 10. Run navigation with a saved map

Stop the SLAM launch, then run:

```bash
ros2 launch mdetect_bringup robot_navigation.launch.py \
  map:=$HOME/mdetect_ws/src/mdetect_bringup/maps/tunnel_map.yaml
```

Use RViz2 **2D Pose Estimate** once to initialize AMCL, then send timed waypoints.

## 11. Topic flow

```text
COIN-D6 cspc_lidar -> /scan -> SLAM Toolbox + costmaps + safety supervisor
/wheel/odometry + /imu/data -> robot_localization
/odometry/filtered -> Nav2
Nav2 controller -> /cmd_vel_nav
velocity smoother -> /cmd_vel
safety supervisor -> /cmd_vel_safe
serial bridge -> Arduino four wheel targets
```

## 12. Recommended test order

1. Test every motor and encoder with the robot raised.
2. Confirm positive ROS yaw is counter-clockwise.
3. Confirm `/wheel/odometry` moves forward on the RViz X axis.
4. Confirm the complete TF tree.
5. Confirm `/scan` overlays correctly on the robot.
6. Build a map manually at very low speed.
7. Test one Nav2 goal.
8. Test two timed waypoints.
9. Test obstacle slowdown and the latched stop.
10. Test USB disconnection and Wi-Fi loss; the Arduino must stop safely.

## ROS 2 Humble COIN-D6 build correction

The bundled CSPC vendor source has been updated for ROS 2 Humble. Its parameters
are declared with typed default values for `port`, `baudrate`, `frame_id`, and
`version`. This prevents the Humble compiler error:

```text
no matching function for call to rclcpp::Node::declare_parameter(...)
```

If an older copy has already been installed into `~/mdetect_ws`, update the
project package and rerun the installer, or use `scripts/fix_coin_d6_humble.sh`.
The installer also skips the unresolved `ament_python` rosdep key; this key is
the package build type and does not prevent the Python package from building.

After a successful build, verify the driver with:

```bash
source /opt/ros/humble/setup.bash
source ~/mdetect_ws/install/setup.bash
ros2 pkg executables cspc_lidar
ros2 launch cspc_lidar lidar_launch.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/mdetect_ws/install/setup.bash
ros2 topic echo /scan --once
```

### Cleanly rebuild only the COIN-D6 driver

Use the bundled helper to avoid stale `AMENT_PREFIX_PATH` and `CMAKE_PREFIX_PATH` warnings:

```bash
cd ~/mDetectRobot
./scripts/rebuild_lidar.sh
```

The vendor SDK has also been cleaned for ROS 2 Humble: parameter types, format strings, signed/unsigned loops, missing returns, unsafe object clearing, and CMake policy warnings are corrected.
