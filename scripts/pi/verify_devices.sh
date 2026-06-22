#!/usr/bin/env bash
set -euo pipefail
LIDAR="${LIDAR_PORT:-/dev/ttyUSB0}"
ARDUINO="${ARDUINO_PORT:-/dev/ttyUSB1}"
fail=0

check_device() {
  local name="$1" dev="$2"
  if [[ ! -c "$dev" ]]; then echo "[FAIL] $name missing: $dev"; fail=1; return; fi
  if [[ ! -r "$dev" || ! -w "$dev" ]]; then
    echo "[FAIL] $name permission denied: $dev (log out/in after joining dialout)"; fail=1; return
  fi
  echo "[PASS] $name device: $dev -> $(readlink -f "$dev")"
  udevadm info --query=property --name="$dev" | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH)=' || true
}

check_device 'COIN-D6 LiDAR' "$LIDAR"
check_device 'Arduino UNO' "$ARDUINO"

if [[ $fail -eq 0 ]]; then
  python3 - "$ARDUINO" <<'PY' || fail=1
import sys, time
import serial
port = sys.argv[1]
try:
    with serial.Serial(port, 500000, timeout=0.1, write_timeout=0.2) as ser:
        time.sleep(2.0)  # UNO may reset when the port opens
        ser.reset_input_buffer()
        ser.write(b'PING\n')
        deadline = time.monotonic() + 3.0
        seen = []
        while time.monotonic() < deadline:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if line:
                seen.append(line)
                if line == 'PONG' or line.startswith(('READY,', 'T,')):
                    print('[PASS] Arduino serial protocol responded:', line[:100])
                    raise SystemExit(0)
        print('[FAIL] Arduino did not answer PING. Received:', seen[-5:])
        raise SystemExit(1)
except Exception as exc:
    print('[FAIL] Arduino serial test:', exc)
    raise SystemExit(1)
PY
fi

[[ -e /dev/coin_d6 ]] && echo "[PASS] Stable LiDAR link: /dev/coin_d6"
[[ -e /dev/arduino_mdetect ]] && echo "[PASS] Stable Arduino link: /dev/arduino_mdetect"
exit "$fail"
