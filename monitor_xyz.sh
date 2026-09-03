#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/ros_env.sh
source "$PROJECT_DIR/scripts/ros_env.sh"
source_ros2
source_ros_file "$PROJECT_DIR/install/setup.bash"

ros2 topic echo /person_position
