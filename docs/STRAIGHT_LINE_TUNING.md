# Straight-line correction

The firmware uses three layers of correction:

1. **Direction-specific motor trims** provide the starting PWM needed by each motor. Motor 4 starts at `0.82` forward and `0.66` reverse because it was reported to run faster.
2. **Independent wheel-speed PID** uses each encoder to make measured speed follow the requested speed.
3. **IMU heading hold** captures the current yaw whenever a straight command begins and makes a small differential correction if the robot starts turning.

Keep the robot still during Arduino startup because the MPU6050 calibrates its gyro.

## Safe wheel test

```bash
bash scripts/pi/test_actuators_on_blocks.sh --confirm-robot-lifted
```

Check that forward commands make all four wheel-speed values positive:

```bash
ros2 topic echo /base/wheel_speeds_mm_s
```

## Floor test

Provide at least 2 m of clear floor:

```bash
bash scripts/pi/straight_line_test.sh --confirm-clear-path
```

## Change motor 4 forward trim

```bash
bash scripts/pi/set_motor_tuning.sh trimf 4 0.80
```

Use a smaller number if motor 4 is still too fast. Use a larger number if it becomes too slow. Repeat for reverse with `trimr`.

## Change heading hold

```bash
bash scripts/pi/set_motor_tuning.sh heading 2.0 22.0
```

The first number is proportional gain. The second is the maximum angular correction in degrees per second. Reduce the gain if the robot oscillates left and right. Increase it gradually if correction is too weak.

Runtime tuning is not stored in EEPROM. After finding good values, copy them into `arduino/autonomous_v12/autonomous_v12.ino` and upload the firmware again.
