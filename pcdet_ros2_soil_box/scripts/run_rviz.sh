#!/usr/bin/env bash

set -euo pipefail

migration_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "$migration_dir/ros2_ws/install/setup.bash" ]]; then
  source "$migration_dir/ros2_ws/install/setup.bash"
fi
exec rviz2
