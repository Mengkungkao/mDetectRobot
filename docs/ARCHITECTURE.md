# mDetect ROS 2 architecture

```text
Ubuntu workstation (ROS 2 Humble desktop)
  RViz2
  SLAM Toolbox or AMCL
  Nav2 A* planner
  Regulated Pure Pursuit controller
  keyboard teleop and waypoint client
                |
                | ROS 2 DDS, ROS_DOMAIN_ID=30
                v
Raspberry Pi Ubuntu Server 22.04
  robot_state_publisher
  cspc_lidar SDK: /dev/ttyUSB0 -> /scan
  mdetect_cmd_mux: manual > teleop > Nav2
  front LiDAR safety gate
  mdetect_serial_bridge: /cmd_vel <-> /dev/ttyUSB1
  /odom, /imu/data, /joint_states, wheel telemetry, TF
                |
                | USB serial, 500000 baud
                v
Arduino UNO
  four encoder inputs
  MPU6050 yaw
  four independent motor PID controllers
  direction-specific motor feed-forward trims
  straight-line IMU heading hold
  one-second brake then release
  500 ms command watchdog and latched emergency stop
```

The Pi owns the physical robot. The workstation owns mapping, localisation, planning and operator interfaces. This follows the same separation used by TurtleBot3 systems.
