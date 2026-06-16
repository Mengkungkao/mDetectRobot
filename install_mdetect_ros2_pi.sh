#!/usr/bin/env bash
# Install the mDetect ROS 2 Humble package into ~/ros2_ws on Raspberry Pi.
#
# Usage:
#   chmod +x install_mdetect_ros2_pi.sh
#   ./install_mdetect_ros2_pi.sh /path/to/mdetect_turtlebot3_ros2_humble.zip
#
# Optional environment variables:
#   ROS_WS=$HOME/ros2_ws
#   ROS_DISTRO=humble
#   ROS_DOMAIN_ID=10

set -Eeuo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"
DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
ZIP_FILE="${1:-$HOME/mdetect_turtlebot3_ros2_humble.zip}"
PACKAGE_NAME="mdetect_robot"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

log() {
    printf '\n\033[1;34m[mdetect-install]\033[0m %s\n' "$*"
}

warn() {
    printf '\n\033[1;33m[warning]\033[0m %s\n' "$*" >&2
}

fail() {
    printf '\n\033[1;31m[error]\033[0m %s\n' "$*" >&2
    exit 1
}

cleanup() {
    if [[ -n "${TMP_DIR:-}" && -d "${TMP_DIR:-}" ]]; then
        rm -rf "$TMP_DIR"
    fi
}
trap cleanup EXIT
trap 'fail "Installation stopped near line $LINENO."' ERR

if [[ "$EUID" -eq 0 ]]; then
    fail "Run this script as your normal Pi user, not with sudo. The script uses sudo only for system packages."
fi

[[ -f "$ZIP_FILE" ]] || fail "ZIP file not found: $ZIP_FILE"
[[ -f "$ROS_SETUP" ]] || fail "ROS 2 ${ROS_DISTRO} is not installed at $ROS_SETUP"

if ! command -v sudo >/dev/null 2>&1; then
    fail "sudo is required to install system dependencies."
fi

log "Installing required tools and Pi-side ROS packages"
sudo apt-get update
sudo apt-get install -y \
    unzip \
    rsync \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-serial \
    "ros-${ROS_DISTRO}-robot-state-publisher" \
    "ros-${ROS_DISTRO}-xacro" \
    "ros-${ROS_DISTRO}-nav2-collision-monitor" \
    "ros-${ROS_DISTRO}-nav2-lifecycle-manager"

# Permit the current user to open Arduino USB serial ports.
if ! id -nG "$USER" | grep -qw dialout; then
    log "Adding $USER to the dialout group"
    sudo usermod -aG dialout "$USER"
    warn "The dialout group change becomes active after logging out and back in, or after rebooting."
fi

log "Preparing rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update

log "Extracting the project package"
TMP_DIR="$(mktemp -d)"
unzip -q "$ZIP_FILE" -d "$TMP_DIR"

PACKAGE_SOURCE="$(find "$TMP_DIR" -type d -path '*/ros2_ws/src/mdetect_robot' -print -quit)"
[[ -n "$PACKAGE_SOURCE" && -f "$PACKAGE_SOURCE/package.xml" ]] || \
    fail "Could not find ros2_ws/src/mdetect_robot/package.xml inside $ZIP_FILE"

mkdir -p "$ROS_WS/src"
TARGET_PACKAGE="$ROS_WS/src/$PACKAGE_NAME"

if [[ -d "$TARGET_PACKAGE" ]]; then
    BACKUP="${TARGET_PACKAGE}.backup.$(date +%Y%m%d_%H%M%S)"
    log "Backing up the existing package to $BACKUP"
    mv "$TARGET_PACKAGE" "$BACKUP"
fi

log "Copying $PACKAGE_NAME into $ROS_WS/src"
mkdir -p "$TARGET_PACKAGE"
rsync -a --delete \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    "$PACKAGE_SOURCE/" "$TARGET_PACKAGE/"

# shellcheck disable=SC1090
source "$ROS_SETUP"

log "Installing package dependencies"
# These three packages are workstation-only. Skipping them keeps the Pi install
# small while retaining robot_state_publisher, serial bridge, and collision monitor.
rosdep install \
    --from-paths "$ROS_WS/src/$PACKAGE_NAME" \
    --ignore-src \
    --rosdistro "$ROS_DISTRO" \
    --skip-keys="rviz2 slam_toolbox nav2_bringup" \
    --reinstall \
    -r -y

log "Building $PACKAGE_NAME"
cd "$ROS_WS"
rm -rf \
    "build/$PACKAGE_NAME" \
    "install/$PACKAGE_NAME"

colcon build \
    --symlink-install \
    --packages-select "$PACKAGE_NAME" \
    --event-handlers console_direct+

BASHRC="$HOME/.bashrc"
ROS_SOURCE_LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"
WS_SOURCE_LINE="source ${ROS_WS}/install/setup.bash"
DOMAIN_LINE="export ROS_DOMAIN_ID=${DOMAIN_ID}"
LOCALHOST_LINE="export ROS_LOCALHOST_ONLY=0"

append_once() {
    local line="$1"
    grep -Fqx "$line" "$BASHRC" 2>/dev/null || printf '%s\n' "$line" >> "$BASHRC"
}

log "Updating $BASHRC"
append_once "$ROS_SOURCE_LINE"
append_once "$WS_SOURCE_LINE"
append_once "$DOMAIN_LINE"
append_once "$LOCALHOST_LINE"

# shellcheck disable=SC1090
source "$ROS_WS/install/setup.bash"

log "Checking the installed package"
ros2 pkg prefix "$PACKAGE_NAME" >/dev/null
ros2 pkg executables "$PACKAGE_NAME"

cat <<DONE

Installation completed successfully.

Workspace:
  $ROS_WS

Package:
  $TARGET_PACKAGE

Start the Pi robot nodes with:
  source $ROS_WS/install/setup.bash
  ros2 launch mdetect_robot robot_bringup.launch.py \\
    arduino_port:=/dev/ttyUSB0 \\
    arduino_baud:=500000

Before starting:
  1. Start the COIN-D6 LiDAR driver and confirm it publishes /scan.
  2. Confirm the Arduino port with: ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
  3. Log out and back in, or reboot, if this script added you to dialout.

ROS_DOMAIN_ID is configured as ${DOMAIN_ID}. Use the same value on the workstation.
DONE
