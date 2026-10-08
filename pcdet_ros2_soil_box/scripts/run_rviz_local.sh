#!/usr/bin/env bash
#
# Start RViz2 for the local (no-venv, Humble) build.

set -euo pipefail

migration_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u  # ROS setup scripts reference possibly-unset variables
source /opt/ros/humble/setup.bash
if [[ -f "$migration_dir/ros2_ws/install/setup.bash" ]]; then
  source "$migration_dir/ros2_ws/install/setup.bash"
fi
set -u

rviz_config="$migration_dir/ros2_ws/install/pcdet_ros2/share/pcdet_ros2/config/pcdet_soil_box.rviz"
if [[ -f "$rviz_config" ]]; then
  echo "Loading pre-configured RViz: $rviz_config"
  exec rviz2 -d "$rviz_config"
fi
exec rviz2
