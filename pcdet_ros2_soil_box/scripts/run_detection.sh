#!/usr/bin/env bash

set -euo pipefail

migration_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="$migration_dir/.venv"
input_topic="${1:-/pointcloud}"
output_topic="${2:-/detection3_d_array}"

if [[ ! -x "$venv_dir/bin/python" ]]; then
  echo "Missing venv. Run scripts/install_and_build.sh first." >&2
  exit 1
fi
if [[ ! -f "$migration_dir/ros2_ws/install/setup.bash" ]]; then
  echo "Missing ROS build. Run scripts/install_and_build.sh first." >&2
  exit 1
fi

source "$venv_dir/bin/activate"
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash
source "$migration_dir/ros2_ws/install/setup.bash"

exec ros2 launch pcdet_ros2 second_soil_box.launch.xml \
  input_topic:="$input_topic" \
  output_topic:="$output_topic"
