#!/usr/bin/env bash
#
# 与 scripts/install_humble_local_overlay.sh 配套的“最小污染启动脚本”。
#
# 特点（都为了不影响 v11 / v15 主链路）：
#   · 强制 source /opt/ros/humble（不使用 jazzy）
#   · 不 source 主 shandong_ws/install/setup.bash
#   · 只把 pcdet_ros2 独立 overlay 的 install/setup.bash 叠加进来
#   · 只把 ~/.local/lib/python3.10/site-packages 追加到 PYTHONPATH，
#     不覆盖系统 site-packages 顺序
#   · 启动前做四重自检：vision_msgs 存在、pcdet 可 import、
#     torch.cuda.is_available()、ros2 能找到 pcdet_ros2 包
#
# 用法：
#   bash scripts/run_detection_humble_local.sh
#   bash scripts/run_detection_humble_local.sh /lidar/points /detection3_d_array
#
# 如果你用的 v15 点云 topic 不是 /pointcloud，这里默认会第一个参数传进去替换；
# 第二个参数是检测输出话题。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MIGRATION_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
OVERLAY_SETUP="$MIGRATION_DIR/ros2_ws/install/setup.bash"
PYTHON_BIN="/usr/bin/python3"

# 和 install_humble_local_overlay.sh 配套的隔离 pydeps 目录：
PYDEPS_DIR="$MIGRATION_DIR/.pydeps"
PYDEPS_SITE="$PYDEPS_DIR/lib/python3.10/site-packages"
mkdir -p "$PYDEPS_SITE"

USERSITE="$("$PYTHON_BIN" -m site --user-site 2>/dev/null || true)"
: "${USERSITE:=$HOME/.local/lib/python3.10/site-packages}"
mkdir -p "$USERSITE"

INPUT_TOPIC="${1:-/pointcloud}"
OUTPUT_TOPIC="${2:-/detection3_d_array}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_DOMAIN_ID

log()  { printf '[pcdet-run] %s\n' "$*"; }
err()  { printf '[pcdet-run][ERR] %s\n' "$*" >&2; }

if [[ ! -f "$ROS_SETUP" ]]; then
  err "找不到 $ROS_SETUP，无法启动 ROS 2 Humble。"
  exit 1
fi
if [[ ! -f "$OVERLAY_SETUP" ]]; then
  err "找不到 pcdet_ros2 overlay: $OVERLAY_SETUP"
  err "请先运行  bash scripts/install_humble_local_overlay.sh  完成安装与构建。"
  exit 1
fi

# --------------------------------------------------------------------
# 激活环境（严格顺序：Humble -> overlay；不加载主 shandong_ws -> 避免污染 v11/v15）
# --------------------------------------------------------------------
set +u
source "$ROS_SETUP"
source "$OVERLAY_SETUP"
set -u

# 让隔离目录内的 pcdet 优先生效；同时保留 ~/.local（如果 v11/v15 或本用户确实有其他包依赖它）
# 注意：PYTHONPATH 直接追加的目录不会自动解析 .pth，显式跑 addsitedir 把 pcdet.pth 展开后条目注入。
PYDEPS_EXTRA_PATHS="$("$PYTHON_BIN" -c 'import site, sys, os; d=sys.argv[1]; old=set(sys.path); site.addsitedir(d); print(os.pathsep.join([p for p in sys.path if p and p not in old]))' "$PYDEPS_SITE" 2>/dev/null || true)"
export PYTHONPATH="$PYDEPS_SITE:$USERSITE:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
if [[ -n "$PYDEPS_EXTRA_PATHS" ]]; then
  export PYTHONPATH="$PYDEPS_EXTRA_PATHS:${PYTHONPATH:-}"
fi

# 让 pcdet CUDA ops *.so 能找到 libc10.so / libtorch_*.so（在 torch 的 lib 目录）
TORCH_LIB_DIR="$("$PYTHON_BIN" -c 'import os,torch; print(os.path.join(os.path.dirname(torch.__file__),"lib"))' 2>/dev/null || true)"
if [[ -n "$TORCH_LIB_DIR" && -d "$TORCH_LIB_DIR" ]]; then
  export LD_LIBRARY_PATH="$TORCH_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi

# --------------------------------------------------------------------
# 运行前自检（任何一项关键缺失直接退出，避免起来之后一堆看不懂的 ImportError）
# --------------------------------------------------------------------
log "启动前自检 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)..."
set +e
PREFLIGHT_OUT="$(
  "$PYTHON_BIN" - <<PY 2>&1
import os, sys
os.environ["ROS_DOMAIN_ID"] = "${ROS_DOMAIN_ID}"
ok = True
# 1) vision_msgs 必须存在（package.xml 的 exec_depend）
try:
    import vision_msgs
    print(f"vision_msgs imported: {vision_msgs.__file__}")
except Exception as e:
    print(f"ERR vision_msgs: {type(e).__name__}: {e}")
    print("HINT: sudo apt install ros-humble-vision-msgs ros-humble-vision-msgs-rviz-plugins")
    ok = False

# 2) pcdet + CUDA ops 必须可用
try:
    import pcdet
    from pcdet.ops.iou3d_nms import iou3d_nms_cuda
    from pcdet.ops.roiaware_pool3d import roiaware_pool3d_cuda
    print(f"pcdet imported: {pcdet.__file__}")
except Exception as e:
    print(f"ERR pcdet: {type(e).__name__}: {e}")
    ok = False

# 3) torch + CUDA（这里不强制 GPU 可用，但会 warn）
try:
    import torch
    print(f"torch: {torch.__version__}  cuda_available={torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"gpu: {torch.cuda.get_device_name(0)}")
    else:
        print("WARN torch.cuda.is_available()=False；检测节点会推理失败。")
except Exception as e:
    print(f"ERR torch: {type(e).__name__}: {e}")
    ok = False

# 4) rclpy / sensor_msgs_py
try:
    import rclpy
    from sensor_msgs_py import point_cloud2
    print("rclpy/sensor_msgs_py: OK")
except Exception as e:
    print(f"ERR rclpy/sensor_msgs_py: {type(e).__name__}: {e}")
    ok = False

print("PREFLIGHT_DONE_OK" if ok else "PREFLIGHT_DONE_FAIL")
sys.exit(0 if ok else 2)
PY
)"
PRE_RC=$?
set -e
echo "$PREFLIGHT_OUT"
if [[ $PRE_RC -ne 0 ]]; then
  err "启动前自检失败，看上面 ERR 行修正后重试。"
  exit 2
fi

set +e
ros2 pkg prefix pcdet_ros2 >/dev/null 2>&1
RC=$?
set -e
if [[ $RC -ne 0 ]]; then
  err "ros2 找不到包 pcdet_ros2，请确认 overlay 构建已完成。"
  exit 3
fi

log "启动检测节点。input_topic=$INPUT_TOPIC  output_topic=$OUTPUT_TOPIC"
log "固定 frame_id / TF 说明：默认期望 input 点云以 base_link 发布；若你使用 odom 世界系，"
log "需要在系统里存在 base_link <-> odom 的 TF（与 README_CN.md 描述一致）。"

set +e
ros2 launch pcdet_ros2 second_soil_box.launch.xml \
  input_topic:="$INPUT_TOPIC" \
  output_topic:="$OUTPUT_TOPIC"
RC=$?
set -e

log "检测节点退出，code=$RC"
exit $RC
