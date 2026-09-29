#!/usr/bin/env bash
#
# 启动 shandong_0.5 的 V15 主控节点。
#
# 为什么要单独写这个脚本：
#   真机上你很可能默认 conda 激活了 python3.14，而 ROS2 Humble 的 rclpy /
#   geometry_msgs / cv_bridge 都是针对系统 python3.10 编译的。如果你用
#   `python3 main.py`，很可能进 conda 的 python3.14，然后 import rclpy 直接报：
#     ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
#   所以这个脚本强制：无论当前 shell 里 python3 指向什么，都用
#     /usr/bin/python3  (3.10.x)  +  /opt/ros/humble/setup.bash  +  ROS_DOMAIN_ID=0
#   启动 main.py。
#
# 用法：
#   chmod +x src/shandong/v15_action_task/shandong_0.5/start_main.sh
#   src/shandong/v15_action_task/shandong_0.5/start_main.sh
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

export ROS_DOMAIN_ID=0
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[start_main.sh] 找不到 /opt/ros/humble/setup.bash，请先安装 ROS2 Humble。" >&2
  exit 2
fi

echo "[start_main.sh] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[start_main.sh] ws = $WS_DIR"
echo "[start_main.sh] use python = $(/usr/bin/python3 --version || true) at $(which /usr/bin/python3 || true)"

cd "$WS_DIR"
exec /usr/bin/python3 -u "src/shandong/v15_action_task/shandong_0.5/main.py" "$@"
