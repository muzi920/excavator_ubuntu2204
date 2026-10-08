#!/usr/bin/env bash
#
# Verify the local (no-venv) Soil+Box installation.
# Checks CUDA, the OpenPCDet extensions in this bundle, and the ROS package.

set -euo pipefail

migration_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

unset PYTHONPATH
set +u  # ROS setup scripts reference possibly-unset variables
source /opt/ros/humble/setup.bash
source "$migration_dir/ros2_ws/install/setup.bash" 2>/dev/null || true
set -u
export PYTHONPATH="$migration_dir/third_party/DSVT:$PYTHONPATH"

python3 - <<'PY'
import sys
import torch
import spconv.pytorch
import pcdet
import rclpy
from sensor_msgs_py import point_cloud2
from pcdet.ops.iou3d_nms import iou3d_nms_cuda
from pcdet.ops.roiaware_pool3d import roiaware_pool3d_cuda

print('Python:', sys.executable)
print('PyTorch:', torch.__version__)
print('CUDA available:', torch.cuda.is_available())
if not torch.cuda.is_available():
    raise SystemExit('CUDA is unavailable')
print('GPU:', torch.cuda.get_device_name(0))
print('OpenPCDet:', pcdet.__file__)
print('OpenPCDet CUDA extensions: OK')
print('sensor_msgs_py: OK')
PY

ros2 pkg prefix pcdet_ros2 >/dev/null
echo "pcdet_ros2 package: OK"
echo "Installation verification: OK"
