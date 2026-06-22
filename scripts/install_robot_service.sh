#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SERVICE=/etc/systemd/system/mdetect-robot.service
sudo tee "$SERVICE" >/dev/null <<SERVICE
[Unit]
Description=mDetect ROS 2 robot bringup
After=network-online.target dev-ttyUSB0.device dev-ttyUSB1.device
Wants=network-online.target

[Service]
Type=simple
User=$USER
Environment=HOME=$HOME
ExecStart=$PROJECT_DIR/scripts/bringup_pi.sh
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
SERVICE
sudo systemctl daemon-reload
sudo systemctl enable mdetect-robot.service
printf 'Installed. Start with: sudo systemctl start mdetect-robot\n'
