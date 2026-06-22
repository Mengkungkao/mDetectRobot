#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
USER_NAME="$USER"
HOME_DIR="$HOME"
SERVICE=/etc/systemd/system/mdetect-robot.service
sudo tee "$SERVICE" >/dev/null <<SERVICE
[Unit]
Description=mDetect ROS 2 robot bringup
After=network-online.target dev-ttyUSB0.device dev-ttyUSB1.device
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
Environment=HOME=$HOME_DIR
Environment=ROS_DOMAIN_ID=30
Environment=ROS_LOCALHOST_ONLY=0
ExecStart=/bin/bash $PROJECT_ROOT/scripts/pi/bringup_robot.sh
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
SERVICE
sudo systemctl daemon-reload
sudo systemctl enable --now mdetect-robot.service
systemctl --no-pager --full status mdetect-robot.service || true
