#!/usr/bin/env bash
exec bash "$(cd "$(dirname "$0")" && pwd)/pi/install_pi.sh" "$@"
