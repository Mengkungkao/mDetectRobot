# USB ports

The requested initial mapping is:

- `/dev/ttyUSB0`: COIN-D6 LiDAR at 230400 baud
- `/dev/ttyUSB1`: Arduino UNO at 500000 baud

Linux numbering can change after a reboot or reconnect. Create stable links while the devices are connected in the correct ports:

```bash
bash scripts/pi/configure_udev.sh /dev/ttyUSB0 /dev/ttyUSB1
```

Expected links:

```text
/dev/coin_d6
/dev/arduino_mdetect
```

The Pi bringup script automatically prefers these stable links and falls back to `/dev/ttyUSB0` and `/dev/ttyUSB1`.
