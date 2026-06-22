#!/usr/bin/env bash
set -euo pipefail
if [[ ! -r /etc/os-release ]]; then echo "Cannot identify Ubuntu version" >&2; exit 1; fi
. /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "This project targets Ubuntu 22.04 (Jammy) with ROS 2 Humble." >&2
  exit 1
fi
sudo apt-get update
sudo apt-get install -y locales software-properties-common curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe
sudo mkdir -p /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
  sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt-get update
