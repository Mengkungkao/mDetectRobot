# COIN-D6 LiDAR setup

The COIN-D6 uses the bundled `cspc_lidar` ROS 2 package. The Raspberry Pi installer copies and builds this driver with the rest of the workspace, installs its USB rule, and configures the driver to publish `/scan` in the `lidar_link` frame.

## USB and serial settings

Default configuration:

```yaml
port: /dev/sc_mini
baudrate: 230400
frame_id: lidar_link
version: 4
```

The configuration file is:

```text
~/mdetect_ws/src/cspc_lidar/params/cspc_lidar.yaml
```

The supplied udev rule recognizes common CH340 and CP210x USB-to-serial adapters and creates `/dev/sc_mini`.

After installation, unplug and reconnect the LiDAR, then check:

```bash
ls -l /dev/sc_mini
readlink -f /dev/sc_mini
```

If the LiDAR and Arduino use identical USB-to-serial chipsets, the generic vendor rule may not distinguish them reliably. In that case, use a stable path from:

```bash
ls -l /dev/serial/by-id/
```

Then set that path as `port` in `cspc_lidar.yaml`.

## Run the LiDAR by itself

```bash
source ~/mdetect_ws/install/setup.bash
ros2 launch cspc_lidar lidar_launch.py
```

Verify the output:

```bash
ros2 topic list | grep scan
ros2 topic info /scan
ros2 topic echo /scan --once
ros2 topic hz /scan
```

Expected output:

```text
Topic: /scan
Type: sensor_msgs/msg/LaserScan
Frame: lidar_link
Nominal rate: approximately 10 Hz
```

## Run with the full robot

The LiDAR starts automatically with both full robot launch files:

```bash
ros2 launch mdetect_bringup robot_slam.launch.py
```

or:

```bash
ros2 launch mdetect_bringup robot_navigation.launch.py \
  map:=$HOME/mdetect_ws/src/mdetect_bringup/maps/tunnel_map.yaml
```

Disable automatic LiDAR startup when testing another scanner or replaying a rosbag:

```bash
ros2 launch mdetect_bringup robot_slam.launch.py start_lidar:=false
```

Use a custom parameter file:

```bash
ros2 launch mdetect_bringup robot_slam.launch.py \
  lidar_params:=/absolute/path/to/cspc_lidar.yaml
```

## Orientation check

Place a clear object directly in front of the robot. In RViz2, the corresponding scan points must appear in front of `base_link`. If the scan is mirrored or rotated, first check the driver's `reversion` setting and then the fixed `lidar_joint` transform in the robot Xacro. Keep the published scan frame and URDF frame name consistent.
