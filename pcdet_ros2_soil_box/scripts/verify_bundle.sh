#!/usr/bin/env bash

set -euo pipefail

migration_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$migration_dir"

required_files=(
  "requirements.txt"
  "ros2_ws/src/pcdet_ros2/package.xml"
  "ros2_ws/src/pcdet_ros2/pcdet_ros2/pcdet_node.py"
  "ros2_ws/src/pcdet_ros2/pcdet_ros2/geometry.py"
  "ros2_ws/src/pcdet_ros2/launch/second_soil_box.launch.xml"
  "ros2_ws/src/pcdet_ros2/config/pcdet_second_soil_box.param.yaml"
  "ros2_ws/src/pcdet_ros2/cfgs/custom_models/second_soil_box.yaml"
  "ros2_ws/src/pcdet_ros2/cfgs/dataset_configs/custom_soil_box.yaml"
  "ros2_ws/src/pcdet_ros2/checkpoints/checkpoint_soil_box_epoch_80.pth"
  "third_party/DSVT/setup.py"
  "third_party/DSVT/pcdet/__init__.py"
)

for required_file in "${required_files[@]}"; do
  [[ -f "$required_file" ]] || {
    echo "Missing required file: $required_file" >&2
    exit 1
  }
done

sha256sum --check SHA256SUMS
compiled_file="$(find third_party/DSVT -type f \( -name '*.so' -o -name '*.o' -o -name '*.pyc' \) -print -quit)"
if [[ -n "$compiled_file" ]]; then
  echo "Unexpected old-machine binary/cache file: $compiled_file" >&2
  exit 1
fi

echo "No-Conda Soil+Box migration folder verification: OK"
