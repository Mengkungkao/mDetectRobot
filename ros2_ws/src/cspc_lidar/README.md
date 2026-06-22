# CSPC COIN-D6 ROS 2 driver

This directory contains the vendor CSPC ROS 2 SDK supplied with the project and adjusted for ROS 2 Humble integration.

Default settings:

- Device: `/dev/sc_mini`
- Baud rate: `230400`
- Topic: `/scan`
- Message: `sensor_msgs/msg/LaserScan`
- TF frame: `lidar_link`
- Driver parameter: `version: 4`

Run only the driver:

```bash
source /opt/ros/humble/setup.bash
source ~/mdetect_ws/install/setup.bash
ros2 launch cspc_lidar lidar_launch.py
```

Override the parameter file:

```bash
ros2 launch cspc_lidar lidar_launch.py \
  params_file:=/absolute/path/to/cspc_lidar.yaml
```

The vendor archive supplied for this project is named `cspc_lidar_sdk_ros2_D4_20250731`. Hardware behaviour must still be verified on the actual COIN-D6 unit.

## Clean build

From the project directory, run:

```bash
./scripts/rebuild_lidar.sh
```

The launch file does not automatically respawn a crashing driver. This keeps the original error visible instead of producing a repeated crash loop. Run the driver on the Raspberry Pi where `/dev/sc_mini` exists; RViz and other workstation tools subscribe to `/scan` over ROS 2 DDS.
