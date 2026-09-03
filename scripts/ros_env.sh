#!/usr/bin/env bash

# Source a ROS 2 installation without pinning this project to a distribution.
# If multiple distros are installed, ROS_DISTRO must identify the desired one.
source_ros_file() {
  local setup_file="$1"
  local restore_nounset=false

  case $- in
    *u*)
      restore_nounset=true
      set +u
      ;;
  esac

  # ROS and colcon generated setup files are not safe under `set -u`.
  # shellcheck disable=SC1090
  source "$setup_file"

  if [ "$restore_nounset" = true ]; then
    set -u
  fi
}

source_ros2() {
  if command -v ros2 >/dev/null 2>&1 && [ -n "${ROS_DISTRO:-}" ]; then
    return 0
  fi

  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source_ros_file "/opt/ros/${ROS_DISTRO}/setup.bash"
    return 0
  fi

  local setups=()
  local setup
  for setup in /opt/ros/*/setup.bash; do
    [ -f "$setup" ] && setups+=("$setup")
  done

  if [ "${#setups[@]}" -eq 1 ]; then
    source_ros_file "${setups[0]}"
    return 0
  fi

  if [ "${#setups[@]}" -eq 0 ]; then
    echo "ERROR: No ROS 2 installation was found under /opt/ros." >&2
  else
    echo "ERROR: Multiple ROS 2 distributions are installed." >&2
    echo "Set ROS_DISTRO first, for example: export ROS_DISTRO=jazzy" >&2
  fi
  return 1
}
