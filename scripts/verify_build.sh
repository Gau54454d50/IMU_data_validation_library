#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  SUDO=sudo
else
  SUDO=
fi

$SUDO apt-get update -y
$SUDO apt-get install -y curl ca-certificates gnupg2 lsb-release

$SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | $SUDO tee /etc/apt/sources.list.d/ros2.list > /dev/null

$SUDO apt-get update -y
$SUDO apt-get install -y ros-jazzy-ros-base

source /opt/ros/jazzy/setup.bash
cmake -S imu_validator -B build
