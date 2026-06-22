#!/usr/bin/env bash
exec bash "$(cd "$(dirname "$0")" && pwd)/scripts/workstation/bringup_navigation.sh" "$@"
