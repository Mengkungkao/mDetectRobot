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

## 2. Install the Raspberry Pi workspace

Copy this whole folder to the Pi, then run:

```bash
cd AutonomousV11_complete
./scripts/install_pi.sh
```

Log out and back in after installation so membership in the `dialout` group takes effect.

Find stable serial device paths:

```bash
ls -l /dev/serial/by-id/
```

Edit:

```bash
nano ~/mdetect_ws/src/mdetect_base/config/base.yaml
```

Set the Arduino path, preferably using `/dev/serial/by-id/...`, then rebuild:

```bash
cd ~/mdetect_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. Start the COIN-D6 LiDAR

The supplied files did not include the COIN-D6 packet protocol or its ROS 2 driver source, so this bundle does not replace the vendor `cspc_lidar` driver. Start your existing driver on the Pi and make sure it publishes:

```text
Topic: /scan
Type:  sensor_msgs/msg/LaserScan
Frame: lidar_link
```

Use these checks:

```bash
ros2 pkg executables cspc_lidar
ros2 topic hz /scan
ros2 topic echo /scan --once
```

If the driver publishes a different topic, remap it to `/scan`. If it publishes another frame name, either change the driver frame to `lidar_link` or update the URDF and configuration consistently.

## 4. Run SLAM + Nav2 on the Pi

After the LiDAR driver is running:

```bash
source ~/mdetect_ws/install/setup.bash
ros2 launch mdetect_bringup robot_slam.launch.py
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
/scan -> SLAM Toolbox + costmaps + safety supervisor
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
