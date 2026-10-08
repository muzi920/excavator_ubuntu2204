#!/usr/bin/env bash

set -euo pipefail

migration_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="$migration_dir/.venv"
ros_setup="/opt/ros/jazzy/setup.bash"
cuda_home="${CUDA_HOME:-/usr/local/cuda-12.8}"
cuda_arch="${TORCH_CUDA_ARCH_LIST:-12.0}"
max_jobs="${MAX_JOBS:-4}"
torch_index="${PYTORCH_INDEX_URL:-https://download.pytorch.org/whl/cu128}"

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS 2 Jazzy not found: $ros_setup" >&2
  exit 1
fi
if ! command -v python3 >/dev/null; then
  echo "python3 is required" >&2
  exit 1
fi
if ! python3 -c 'import sys; raise SystemExit(sys.version_info[:2] != (3, 12))'; then
  echo "Python 3.12 is required" >&2
  exit 1
fi
if [[ ! -x "$cuda_home/bin/nvcc" ]]; then
  echo "CUDA compiler not found: $cuda_home/bin/nvcc" >&2
  exit 1
fi
if [[ ! -x /usr/bin/colcon ]]; then
  echo "colcon not found; install python3-colcon-common-extensions" >&2
  exit 1
fi

python3 -m venv "$venv_dir"
source "$venv_dir/bin/activate"
python -m pip install --upgrade pip setuptools wheel
python -m pip install \
  torch==2.11.0 \
  torchvision==0.26.0 \
  --index-url "$torch_index"
python -m pip install -r "$migration_dir/requirements.txt"

CUDA_HOME="$cuda_home" \
TORCH_CUDA_ARCH_LIST="$cuda_arch" \
MAX_JOBS="$max_jobs" \
python -m pip install --no-build-isolation -e "$migration_dir/third_party/DSVT"

unset PYTHONPATH
source "$ros_setup"
venv_site="$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
cd "$migration_dir/ros2_ws"
PYTHONPATH="$venv_site:/usr/lib/python3/dist-packages:${PYTHONPATH:-}" \
  "$venv_dir/bin/python" /usr/bin/colcon build \
  --symlink-install \
  --packages-select pcdet_ros2

echo "Build complete"
echo "Next: bash scripts/verify_install.sh"
