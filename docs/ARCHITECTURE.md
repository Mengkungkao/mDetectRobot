# Runtime architecture

```text
Ubuntu workstation
  RViz2
  waypoint input / monitoring
          |
          | ROS 2 DDS over LAN, ROS_DOMAIN_ID=42
          v
Raspberry Pi Ubuntu Server
  COIN-D6 vendor driver -> /scan
  robot_state_publisher
  serial_bridge -> wheel odometry + IMU
  robot_localization -> /odometry/filtered and odom->base_footprint
  SLAM Toolbox -> /map and map->odom
  Nav2 SmacPlanner2D A* -> global path
  Regulated Pure Pursuit -> cmd_vel_nav
  Nav2 velocity smoother -> /cmd_vel
  safety_supervisor -> /cmd_vel_safe
  serial_bridge -> four wheel targets
          |
          | USB serial 500000 baud
          v
Arduino Uno
  four encoders + MPU6050
  four wheel PID controllers
  motor shield
  watchdog + emergency-stop latch
```

TF chain:

```text
map -> odom -> base_footprint -> base_link -> lidar_link
                                       `-> imu_link
                                       `-> wheel links
```
