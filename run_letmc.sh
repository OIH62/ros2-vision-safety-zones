#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/ros_env.sh
source "$PROJECT_DIR/scripts/ros_env.sh"
source_ros2

if [ ! -f "$PROJECT_DIR/install/setup.bash" ]; then
  echo "ERROR: Workspace is not built. Run ./setup.sh first." >&2
  exit 1
fi
source_ros_file "$PROJECT_DIR/install/setup.bash"

ros2 launch person_zone_cpp pipeline.launch.py "$@"
