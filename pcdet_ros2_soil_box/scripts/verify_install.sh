#!/usr/bin/env bash

set -euo pipefail

migration_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="$migration_dir/.venv"

[[ -x "$venv_dir/bin/python" ]] || {
  echo "Missing venv. Run scripts/install_and_build.sh first." >&2
  exit 1
}

source "$venv_dir/bin/activate"
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash
source "$migration_dir/ros2_ws/install/setup.bash"

python - <<'PY'
import torch
import spconv.pytorch
import pcdet
import rclpy
from sensor_msgs_py import point_cloud2
from pcdet.ops.iou3d_nms import iou3d_nms_cuda
from pcdet.ops.roiaware_pool3d import roiaware_pool3d_cuda

print('Python:', __import__('sys').executable)
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
