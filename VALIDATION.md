# Validation record

Completed static checks for this bundle:

- Parsed every Python node and launch file successfully.
- Parsed all ROS 2 YAML parameter files successfully.
- Parsed every `package.xml` and the robot Xacro as XML.
- Confirmed the EKF sensor vectors contain 15 entries and its process-noise matrix contains 225 entries.
- Confirmed the serial telemetry field count matches between Arduino and Raspberry Pi code.
- Confirmed the Nav2 planner is `nav2_smac_planner/SmacPlanner2D` and the controller is Regulated Pure Pursuit.
- Checked all shell scripts with `bash -n`.

Hardware-dependent checks still required on the actual robot:

- Compile the sketch for Arduino Uno with the installed QGPMaker, PinChangeInterrupt and MPU6050_tockn libraries.
- Build the ROS 2 workspace under Ubuntu 22.04 / ROS 2 Humble using `colcon build`.
- Verify motor direction, encoder direction, yaw sign, wheel diameter and track width.
- Verify the COIN-D6 driver publishes a valid `/scan` in `lidar_link`.
- Tune wheel PID, feed-forward factors, costmap footprint and Nav2 controller settings at low speed.

The current execution environment did not contain ROS 2 Humble, `colcon`, the AVR compiler or Arduino CLI, so a real ROS/Arduino compile and hardware test could not be performed here.
