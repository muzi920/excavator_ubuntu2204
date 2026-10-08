#!/usr/bin/env bash
#
# Run the Soil+Box detection node on THIS machine without a venv.
# Uses the system Python 3.10 + ROS 2 Humble + the precompiled OpenPCDet
# CUDA extensions already present in third_party/DSVT (no nvcc required).
#
# Usage:
#   bash scripts/run_detection_local.sh [<input_topic>] [<output_topic>]

set -euo pipefail

migration_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
input_topic="${1:-/pointcloud}"
output_topic="${2:-/detection3_d_array}"

if [[ ! -f "$migration_dir/ros2_ws/install/setup.bash" ]]; then
  echo "Missing ROS build. Run the local colcon build first:" >&2
  echo "  cd \"$migration_dir/ros2_ws\" && unset PYTHONPATH && \\" >&2
  echo "  source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select pcdet_ros2" >&2
  exit 1
fi

unset PYTHONPATH
set +u  # ROS setup scripts reference possibly-unset variables
source /opt/ros/humble/setup.bash
source "$migration_dir/ros2_ws/install/setup.bash"
set -u

# Prefer this bundle's own OpenPCDet (already has the compiled .so ops).
export PYTHONPATH="$migration_dir/third_party/DSVT:$PYTHONPATH"

exec ros2 launch pcdet_ros2 second_soil_box.launch.xml \
  input_topic:="$input_topic" \
  output_topic:="$output_topic"
