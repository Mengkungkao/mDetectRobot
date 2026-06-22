# Validation record

Completed static checks for this bundle:

- Parsed every Python node and launch file successfully.
- Parsed all ROS 2 YAML parameter files successfully.
- Parsed every `package.xml` and the robot Xacro as XML.
- Checked every shell script with `bash -n`.
- Confirmed the bundled `cspc_lidar` package contains its executable source, launch file, parameters, USB rule and PCL dependencies.
- Confirmed the default COIN-D6 configuration uses `/dev/sc_mini`, `230400` baud, `/scan`, and frame `lidar_link`.
- Confirmed `robot_bringup.launch.py`, `robot_slam.launch.py`, and `robot_navigation.launch.py` pass the `start_lidar` and `lidar_params` options correctly.
- Confirmed the EKF sensor vectors contain 15 entries and its process-noise matrix contains 225 entries.
- Confirmed the serial telemetry field count matches between Arduino and Raspberry Pi code.
- Confirmed the Nav2 planner is `nav2_smac_planner/SmacPlanner2D` and the controller is Regulated Pure Pursuit.

Hardware-dependent checks still required on the actual robot:

- Build the full workspace under Ubuntu 22.04 / ROS 2 Humble using `colcon build`.
- Confirm the vendor CSPC SDK communicates with the actual COIN-D6 using `version: 4`.
- Verify `/dev/sc_mini` resolves to the LiDAR and not another CH340/CP210x device.
- Verify `/scan` is approximately 10 Hz and uses `lidar_link`.
- Verify the scan orientation overlays correctly with the robot model in RViz2.
- Compile the sketch for Arduino Uno with the installed QGPMaker, PinChangeInterrupt and MPU6050_tockn libraries.
- Verify motor direction, encoder direction, yaw sign, wheel diameter and track width.
- Tune wheel PID, feed-forward factors, costmap footprint and Nav2 controller settings at low speed.

The current execution environment did not contain ROS 2 Humble, `colcon`, the AVR compiler or Arduino CLI, so a real ROS/Arduino compile and hardware test could not be performed here.
