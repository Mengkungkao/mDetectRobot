#!/usr/bin/env bash
exec bash "$(cd "$(dirname "$0")" && pwd)/scripts/pi/bringup_robot.sh" "$@"
