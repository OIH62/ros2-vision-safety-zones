#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

# shellcheck source=scripts/ros_env.sh
source "$PROJECT_DIR/scripts/ros_env.sh"
source_ros2

for command_name in rosdep colcon; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "ERROR: Required command is missing: $command_name" >&2
    exit 1
  fi
done

ROS_PYTHON="${ROS_PYTHON_EXECUTABLE:-/usr/bin/python3}"
if [ ! -x "$ROS_PYTHON" ]; then
  echo "ERROR: ROS Python was not found: $ROS_PYTHON" >&2
  echo "Set ROS_PYTHON_EXECUTABLE to the Python used by this ROS install." >&2
  exit 1
fi

if [ ! -f .venv/bin/activate ]; then
  venv_args=(--system-site-packages)
  if [ -d .venv ]; then
    echo "Recreating incomplete ROS-aware Python virtual environment..."
    venv_args+=(--clear)
  else
    echo "Creating a ROS-aware Python virtual environment..."
  fi
  "$ROS_PYTHON" -m venv "${venv_args[@]}" .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate

echo "Installing Python dependencies..."
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

echo "Installing ROS dependencies for ROS_DISTRO=${ROS_DISTRO}..."
rosdep install --from-paths src --ignore-src -r -y --rosdistro "$ROS_DISTRO"

echo "Building workspace..."
python -m colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  "-DPython3_EXECUTABLE=$PROJECT_DIR/.venv/bin/python"

echo
echo "Build complete for ROS 2 ${ROS_DISTRO}."
echo "Start an existing camera driver, then run: ./run_letmc.sh"
