#!/usr/bin/env bash
exec bash "$(cd "$(dirname "$0")" && pwd)/scripts/pi/verify_running.sh" "$@"
