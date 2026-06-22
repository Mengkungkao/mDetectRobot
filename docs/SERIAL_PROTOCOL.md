# Arduino serial protocol

Baud rate: `500000`, ASCII, one command per line.

## Raspberry Pi to Arduino

- `V,seq,m1,m2,m3,m4` — signed wheel linear speeds in mm/s.
- `B,seq,hold_ms` — brake for the requested duration, then release.
- `S,seq` — brake for one second, then release.
- `E,seq` — latch emergency stop, brake for one second, then release while remaining latched.
- `C,seq` — clear the emergency-stop latch. The safety node must also be clear.
- `R,seq` — reset all encoder counts and set the current MPU6050 yaw as zero.
- `P,seq,motor,kp,ki,kd` — change one wheel PID at runtime.
- `Q,seq` — request immediate telemetry.

Motor grouping used by the skid-steer mixer:

- Motor 1: front-right
- Motor 2: front-left
- Motor 3: rear-left
- Motor 4: rear-right

Motors 1 and 4 form the right side. Motors 2 and 3 form the left side.

## Arduino to Raspberry Pi

`T,seq,millis,e1,e2,e3,e4,v1,v2,v3,v4,yaw_cdeg,gyro_cdeg_s,state,fault`

- Encoder values are cumulative, direction-corrected counts.
- Wheel speeds are signed mm/s.
- Yaw is ROS convention: positive counter-clockwise.
- State: `0 released`, `1 running`, `2 braking`, `3 e-stop`, `4 watchdog`, `5 fault`.
- Fault bits: `1 watchdog`, `2 emergency stop`, `4 parser error`.

Acknowledgement: `A,seq,code`, where `0` means accepted.
