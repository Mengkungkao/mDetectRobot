# Validation Status

## Completed before packaging

- All Python source and launch files parsed successfully with Python AST.
- All Bash installers, bringup scripts and verification scripts passed `bash -n`.
- All ROS `package.xml` files parsed as valid XML.
- All YAML parameter and waypoint files parsed successfully.
- The Arduino sketch contains exactly one forward trim array and one reverse trim array.
- The missing `PWM_FORWARD_SCALE` definition from v1 is fixed.
- The supplied CSPC SDK is included as the `cspc_lidar` ROS 2 package.
- The CSPC wrapper no longer depends on PCL or `pcl_conversions`, avoiding the earlier build failure and reducing Raspberry Pi dependencies.
- Default ports are fixed to `/dev/ttyUSB0` for LiDAR and `/dev/ttyUSB1` for Arduino.
- No SHA256 verification step is present.

## Hardware validation to run on the actual robot

1. `./scripts/verify_pi.sh`
2. `./scripts/bringup_pi.sh`
3. In another terminal: `./scripts/verify_pi.sh --live`
4. With the robot lifted safely: `./scripts/test_actuators_on_blocks.sh`
5. On the workstation: `./scripts/verify_workstation.sh --network`
6. Start mapping: `./scripts/bringup_slam.sh`

Hardware serial data, motor rotation and LiDAR scans cannot be physically tested inside the packaging environment; the included scripts perform those checks on the Raspberry Pi and robot.
