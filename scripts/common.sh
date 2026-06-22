#!/usr/bin/env bash
set -euo pipefail

MD_DOMAIN_ID="${MD_DOMAIN_ID:-30}"
MD_WS="${MD_WS:-$HOME/mdetect_ws}"

log() { printf '\033[1;34m[mdetect]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[mdetect warning]\033[0m %s\n' "$*" >&2; }
die() { printf '\033[1;31m[mdetect error]\033[0m %s\n' "$*" >&2; exit 1; }

source_ros() {
  [[ -f /opt/ros/humble/setup.bash ]] || die "ROS 2 Humble is not installed. Run the appropriate installer first."
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ -f "$MD_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "$MD_WS/install/setup.bash"
  fi
  set -u
  export ROS_DOMAIN_ID="$MD_DOMAIN_ID"
  export ROS_LOCALHOST_ONLY=0
}

check_ubuntu_jammy() {
  [[ -r /etc/os-release ]] || die "Cannot identify Ubuntu version."
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_CODENAME:-}" == "jammy" ]] || \
    die "This project targets Ubuntu 22.04 Jammy with ROS 2 Humble. Detected: ${PRETTY_NAME:-unknown}."
}

configure_ros_environment() {
  local bashrc="$HOME/.bashrc"
  local begin='# >>> mdetect ros2 >>>'
  local end='# <<< mdetect ros2 <<<'
  if grep -Fq "$begin" "$bashrc" 2>/dev/null; then
    sed -i "/$begin/,/$end/d" "$bashrc"
  fi
  cat >> "$bashrc" <<ENV
$begin
source /opt/ros/humble/setup.bash
if [ -f "$MD_WS/install/setup.bash" ]; then source "$MD_WS/install/setup.bash"; fi
export ROS_DOMAIN_ID=$MD_DOMAIN_ID
export ROS_LOCALHOST_ONLY=0
$end
ENV
}

repair_duplicate_ros_sources() {
  local official=/etc/apt/sources.list.d/ros2.sources
  if [[ -e "$official" ]]; then
    while IFS= read -r duplicate; do
      [[ "$duplicate" == "$official" ]] && continue
      warn "Removing duplicate legacy ROS source: $duplicate"
      sudo rm -f "$duplicate"
    done < <(sudo grep -l 'packages\.ros\.org/ros2/ubuntu' /etc/apt/sources.list.d/*.list 2>/dev/null || true)
  fi
}

ensure_ros_apt_source() {
  repair_duplicate_ros_sources
  sudo apt-get update
  if apt-cache show ros-humble-ros-base >/dev/null 2>&1; then
    return
  fi

  log "Configuring the official ROS apt source package"
  sudo apt-get install -y curl software-properties-common ca-certificates
  sudo add-apt-repository -y universe

  local version codename deb url
  codename="$(. /etc/os-release && printf '%s' "$VERSION_CODENAME")"
  version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p' | head -1)"
  [[ -n "$version" ]] || die "Could not determine the ros-apt-source release."
  deb="/tmp/ros2-apt-source.deb"
  url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version#v}.${codename}_all.deb"
  curl -fL "$url" -o "$deb"
  sudo dpkg -i "$deb"
  sudo apt-get update
}
