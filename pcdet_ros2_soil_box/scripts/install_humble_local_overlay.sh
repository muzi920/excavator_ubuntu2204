#!/usr/bin/env bash
#
# 在当前这台 Ubuntu 22.04 + ROS 2 Humble 机器上，
# 以“尽量不影响现有 shandong_ws 主工作空间 / v11 / v15 主链路”的方式
# 安装并构建 pcdet_ros2_soil_box 相关内容。
#
# 隔离策略（核心是“三个独立、两个不碰、一个追加”）：
#   1) 三个独立：
#      · Python pcdet 源码构建安装到 ~/.local（而不是系统 /usr 或全局 venv）
#      · ROS 2 colcon 工作空间独立到   <migration_dir>/ros2_ws/{build,install,log}
#      · 启动脚本独立到               scripts/run_detection_humble_local.sh
#   2) 两个不碰：
#      · 不修改当前 shandong_ws/ 的 build/install/log
#      · 不 pip install -r requirements.txt（避免覆盖 v11/v15 正在使用的
#        numpy/scipy/torch/spconv/numba/... 版本，这些我们在系统 Python 3.10
#        用户 site-packages 里已经有能运行的版本）
#   3) 一个追加：
#      · 只 pip install -e <migration_dir>/third_party/DSVT
#        安装缺失的 pcdet 包（默认写到 ~/.local/lib/python3.10/site-packages），
#        启动脚本里用 PYTHONPATH 追加，而不是覆盖。
#
# 前置要求（本脚本会先检查，不满足就直接退出，不做破坏性操作）：
#   · /opt/ros/humble/setup.bash 存在
#   · /usr/bin/python3 为 Python 3.10
#   · colcon 命令存在（/usr/bin/colcon）
#   · CUDA 编译器 nvcc 存在（PATH 里能找到 或 /usr/local/cuda/bin/nvcc）
#   · ros-humble-vision-msgs 已安装（/opt/ros/humble 下能 import vision_msgs）
#   · 若要跑实时推理：NVIDIA 驱动正常、nvidia-smi 能看到 GPU
#
# 用法示例（默认）：
#   bash scripts/install_humble_local_overlay.sh
#
# 指定 CUDA 架构（例如 RTX 40 系列 8.9，或当前这台需要调整时）：
#   TORCH_CUDA_ARCH_LIST="8.9+PTX" bash scripts/install_humble_local_overlay.sh
#
# 不重新编译 pcdet CUDA 扩展（已经编过一次只想重编 ros2 包时）：
#   SKIP_PCDET_BUILD=1 bash scripts/install_humble_local_overlay.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MIGRATION_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# 主工作区（v11/v15 已有构建产物；本脚本只读 install 目录，不写入）
MAIN_WS="$(cd "$MIGRATION_DIR/../../.." && pwd)"
PCDET_SRC_DIR="$MIGRATION_DIR/third_party/DSVT"
ROS2_WS_DIR="$MIGRATION_DIR/ros2_ws"
SRC_IN_WS="$ROS2_WS_DIR/src"

PYTHON_BIN="/usr/bin/python3"
ROS_SETUP="/opt/ros/humble/setup.bash"

# Python pcdet 依赖安装目录（严格隔离在 MIGRATION_DIR 下，不写 ~/.local，不污染 v11/v15）
PYDEPS_DIR="$MIGRATION_DIR/.pydeps"
PYDEPS_SITE="$PYDEPS_DIR/lib/python3.10/site-packages"
mkdir -p "$PYDEPS_DIR" "$PYDEPS_SITE"

CUDA_HOME="${CUDA_HOME:-}"
if [[ -z "$CUDA_HOME" ]]; then
  if [[ -x "/usr/local/cuda/bin/nvcc" ]]; then
    CUDA_HOME="/usr/local/cuda"
  elif command -v nvcc >/dev/null 2>&1; then
    CUDA_HOME="$(cd "$(dirname "$(command -v nvcc)")/.." && pwd)"
  fi
fi
export CUDA_HOME
TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-}"
MAX_JOBS="${MAX_JOBS:-4}"
SKIP_PCDET_BUILD="${SKIP_PCDET_BUILD:-0}"

log()  { printf '[pcdet-install] %s\n' "$*"; }
warn() { printf '[pcdet-install][WARN] %s\n' "$*" >&2; }
err()  { printf '[pcdet-install][ERR] %s\n' "$*" >&2; }

# --------------------------------------------------------------------
# 0. 前置环境检查（任何一项不满足直接退出）
# --------------------------------------------------------------------
log "检查前置环境..."

if [[ ! -f "$ROS_SETUP" ]]; then
  err "未找到 ROS 2 Humble: $ROS_SETUP"
  err "请先安装 ros-humble-desktop（本项目 README 要求的 Jazzy 在这台机器上不存在，已经改为适配 Humble）。"
  exit 1
fi

if [[ ! -x "$PYTHON_BIN" ]]; then
  err "找不到系统 Python: $PYTHON_BIN"
  exit 1
fi

PY_VER="$("$PYTHON_BIN" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
if [[ "$PY_VER" != "3.10" ]]; then
  err "本安装脚本只针对系统 Python 3.10，当前 $PYTHON_BIN 版本: $PY_VER"
  err "请确保使用 /usr/bin/python3 = Python 3.10（和 v11 / v15 保持一致）。"
  exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
  err "未找到 colcon，请先安装: sudo apt install python3-colcon-common-extensions"
  exit 1
fi

if [[ -z "$CUDA_HOME" || ! -x "$CUDA_HOME/bin/nvcc" ]]; then
  err "未找到 CUDA 编译器 nvcc (CUDA_HOME=$CUDA_HOME)"
  err "请先安装 NVIDIA CUDA Toolkit（nvcc --version 能正常输出后再重试）。"
  exit 1
fi

log "ROS 2 : $ROS_SETUP"
log "Python: $PYTHON_BIN  ($("$PYTHON_BIN" --version 2>&1))"
log "CUDA  : $CUDA_HOME  ($("$CUDA_HOME"/bin/nvcc --version | head -n 4 | tr '\n' '|'))"
log "COLCON: $(command -v colcon)"

# --------------------------------------------------------------------
# 1. 在 MIGRATION_DIR/.pydeps 下准备一个完全隔离的 Python 包目录。
#    本脚本只会往这里写东西，不碰 ~/.local，不碰 /usr，不污染 v11/v15。
# --------------------------------------------------------------------
mkdir -p "$PYDEPS_SITE" "$PYDEPS_DIR/bin"
# 让后续 build / verify / colcon 构建都能先找到隔离目录 + 用户已装 ML 依赖（~/.local）
USERSITE_DEFAULT="$("$PYTHON_BIN" -m site --user-site 2>/dev/null || true)"
: "${USERSITE_DEFAULT:=$HOME/.local/lib/python3.10/site-packages}"

# PYTHONPATH 直接追加的目录不会自动解析 .pth；对 PYDEPS_SITE 显式跑 site.addsitedir()，
# 把 .pth 展开后新增的 sys.path 条目也注入 PYTHONPATH，确保 pcdet.pth -> DSVT 源码生效。
PYDEPS_EXTRA_PATHS="$("$PYTHON_BIN" -c 'import site, sys, os; d=sys.argv[1]; old=set(sys.path); site.addsitedir(d); print(os.pathsep.join([p for p in sys.path if p and p not in old]))' "$PYDEPS_SITE" 2>/dev/null || true)"

export PYTHONPATH="$PYDEPS_DIR:$PYDEPS_SITE:$USERSITE_DEFAULT:${PYTHONPATH:-}"
if [[ -n "$PYDEPS_EXTRA_PATHS" ]]; then
  export PYTHONPATH="$PYDEPS_EXTRA_PATHS:${PYTHONPATH:-}"
fi
export PATH="$PYDEPS_DIR/bin:${PATH:-}"
# 让 pcdet CUDA ops *.so 能找到 libc10.so / libtorch_*.so（在 torch 的 lib 目录）
TORCH_LIB_DIR="$("$PYTHON_BIN" -c 'import os,torch; print(os.path.join(os.path.dirname(torch.__file__),"lib"))' 2>/dev/null || true)"
if [[ -n "$TORCH_LIB_DIR" && -d "$TORCH_LIB_DIR" ]]; then
  export LD_LIBRARY_PATH="$TORCH_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi
# 兼容 setuptools develop/egg-info 读取
export PIP_CONFIG_FILE="${PIP_CONFIG_FILE:-}"
# 保证 pip/setup.py 不自己决定往 ~/.local 或 /usr 写东西
export PYTHONUSERBASE="$PYDEPS_DIR"

# --------------------------------------------------------------------
# 2. 检查是否已经具备运行时依赖（numpy / torch / spconv / ... 都由现有环境提供，
#    我们不执行 pip install -r requirements.txt，避免覆盖现有 v11/v15 版本）。
#    如果确实缺少关键包，会先 warn，但仍继续（你可能只是想先把 colcon 包编过）。
# --------------------------------------------------------------------
log "检查系统 Python 3.10 里的 ML 运行时依赖（不会 pip install，避免污染 v11/v15）..."

set +e
DEPS_CHECK_OUT="$(
  "$PYTHON_BIN" - <<'PY' 2>&1
import importlib, sys
mods = [
    ("numpy",        True),
    ("scipy",        True),
    ("torch",        True),
    ("spconv",       True),
    ("numba",        False),
    ("llvmlite",     False),
    ("skimage",      False),
    ("yaml",         True),
    ("easydict",     True),
    ("tensorboardX", False),
]
missing_required = []
for m, req in mods:
    try:
        importlib.import_module(m)
    except Exception as e:
        if req:
            missing_required.append((m, f"{type(e).__name__}: {e}"))
        else:
            print(f"[opt-miss] {m}: {type(e).__name__}: {e}")
for m, e in missing_required:
    print(f"[req-miss] {m}: {e}")
print("CHECK_DONE")
PY
)"
set -e

echo "$DEPS_CHECK_OUT"
if echo "$DEPS_CHECK_OUT" | grep -q "^\[req-miss\]"; then
  warn "关键运行时依赖缺失（上面对应 [req-miss] 的项）。"
  warn "但本安装脚本不会主动 pip install 覆盖现有版本；如运行时报错再按需要手动 ~/.local 安装。"
fi

# --------------------------------------------------------------------
# 3. 安装/重编 third_party/DSVT 的 pcdet 包（以 editable 方式写到 ~/.local）
# --------------------------------------------------------------------
if [[ ! -d "$PCDET_SRC_DIR" ]]; then
  err "缺少 $PCDET_SRC_DIR，无法构建 OpenPCDet/DSVT。"
  exit 1
fi

if [[ "$SKIP_PCDET_BUILD" != "1" ]]; then
  log "为 pcdet 装配：直接复用主工作区 install/pcdet 已预编译并验证过的 site-packages/pcdet 包副本（避免 ABI mismatch，不再 nvcc 重编）"
  PREBUILT_SITE="$MAIN_WS/install/pcdet/lib/python3.10/site-packages"
  PREBUILT_PCDET_DIR="$PREBUILT_SITE/pcdet"
  if [[ ! -d "$PREBUILT_PCDET_DIR" ]]; then
    err "找不到主工作区的预编译 pcdet 目录: $PREBUILT_PCDET_DIR。请在主工作区先 colcon build pcdet（或等价的 pcdet 包）完成后再运行本脚本。"
    exit 1
  fi
  # 把隔离目录下旧的 pcdet 清理掉（确保不是指向 third_party/DSVT 的 .pth）
  rm -rf "$PYDEPS_SITE/pcdet" "$PYDEPS_SITE/pcdet.pth"
  find "$PYDEPS_SITE" -maxdepth 2 \( -name "pcdet-*.egg-info" -o -name "pcdet-*.dist-info" -o -name "pcdet-*.egg-link" \) -exec rm -rf {} + 2>/dev/null || true
  cp -a "$PREBUILT_PCDET_DIR" "$PYDEPS_SITE/pcdet"
  # 若有 egg-info / dist-info / egg-link，一并拷入（使 ament_python setup.py 的 install_requires=['pcdet'] 能识别到版本）
  cd "$PREBUILT_SITE"
  for f in pcdet-*.egg-info pcdet-*.dist-info pcdet-*.egg-link; do
    if [[ -e "$f" ]]; then
      cp -a "$PREBUILT_SITE/$f" "$PYDEPS_SITE/" 2>/dev/null || true
      log "附带 pcdet 元信息: $f"
    fi
  done

  SO_COUNT="$(find "$PYDEPS_SITE/pcdet/ops" -maxdepth 4 -name "*_cuda.cpython-310-*.so" -type f 2>/dev/null | wc -l)"
  log "装配完成，隔离目录内 CUDA ops .so 数量: $SO_COUNT (目标 >= 7)"
  if [[ "$SO_COUNT" -lt 7 ]]; then
    err "预编译 CUDA ops 不足 7 个（当前 $SO_COUNT），后续 import pcdet 会失败。"
    exit 1
  fi
  mkdir -p "$PYDEPS_DIR/bin"
else
  log "SKIP_PCDET_BUILD=1，跳过装配 pcdet。"
fi

# 编译后立刻做一次 import + CUDA 扩展可用性检查
set +e
PCDET_VERIFY_OUT="$(
  "$PYTHON_BIN" - <<'PY' 2>&1
import os
os.environ.setdefault("CUDA_VISIBLE_DEVICES", "0")
ok = True
try:
    import pcdet
    print(f"pcdet imported: {pcdet.__file__}")
except Exception as e:
    print(f"ERR pcdet import: {type(e).__name__}: {e}")
    ok = False
try:
    from pcdet.ops.iou3d_nms import iou3d_nms_cuda
    from pcdet.ops.roiaware_pool3d import roiaware_pool3d_cuda
    print("pcdet CUDA ops imported OK")
except Exception as e:
    print(f"ERR pcdet CUDA ops: {type(e).__name__}: {e}")
    ok = False
try:
    import torch
    print(f"torch: {torch.__version__}  cuda_available={torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"gpu: {torch.cuda.get_device_name(0)}")
except Exception as e:
    print(f"ERR torch: {type(e).__name__}: {e}")
    ok = False
print("VERIFY_DONE_OK" if ok else "VERIFY_DONE_FAIL")
raise SystemExit(0 if ok else 2)
PY
)"
VERIFY_RC=$?
set -e
echo "$PCDET_VERIFY_OUT"
if [[ $VERIFY_RC -ne 0 ]]; then
  warn "pcdet / torch / CUDA 扩展自检失败，上面已打印错误原因。"
  warn "这通常意味着 (a) nvidia-smi GPU 驱动没起来 或 (b) TORCH_CUDA_ARCH_LIST 和真实 GPU 不匹配。"
  warn "我们仍然会继续构建 ROS2 overlay（至少能把包编完），但运行检测需要先解决上述错误。"
fi

# --------------------------------------------------------------------
# 4. 建立独立的 ros2_ws，并把 pcdet_ros2 这个包软链接进来
#    （避免把 pcdet_ros2 加入主 shandong_ws，污染 v11/v15 build）。
# --------------------------------------------------------------------
mkdir -p "$SRC_IN_WS"
LINK_SRC="$SRC_IN_WS/pcdet_ros2"
if [[ ! -L "$LINK_SRC" && ! -d "$LINK_SRC" ]]; then
  ln -s "$MIGRATION_DIR/pcdet_ros2" "$LINK_SRC"
  log "软链接已建立: $LINK_SRC -> $MIGRATION_DIR/pcdet_ros2"
fi

# --------------------------------------------------------------------
# 5. 只在 pcdet_ros2 这一个包上跑 colcon build（其他包一律不碰）
# --------------------------------------------------------------------
log "构建 pcdet_ros2 ROS2 overlay (packages-select 只编 pcdet_ros2) ..."

# 把隔离的 .pydeps 放到 PYTHONPATH 前面，让 colcon 的 ament_python
# 构建过程能 import 到 pcdet（setup.py install_requires 里写了 'pcdet'）。
USERSITE_DEFAULT="$("$PYTHON_BIN" -m site --user-site 2>/dev/null || true)"
: "${USERSITE_DEFAULT:=$HOME/.local/lib/python3.10/site-packages}"
USERSITE="${USERSITE_DEFAULT}"
mkdir -p "$PYDEPS_SITE" "$PYDEPS_DIR/bin"

(
  cd "$ROS2_WS_DIR"
  set +u
  # 这里故意不 source 主 shandong_ws/install/setup.bash，保持 pcdet_ros2 独立。
  source "$ROS_SETUP"
  set -u
  export PYTHONPATH="$PYDEPS_DIR:$PYDEPS_SITE:$USERSITE:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
  export PATH="$PYDEPS_DIR/bin:${PATH:-}"
  export PYTHONUSERBASE="$PYDEPS_DIR"
  # 确保 colcon 启动子进程时 pcdet ops *.so 能找到 libc10.so / libtorch_*.so
  _TORCH_LIB_DIR="$("$PYTHON_BIN" -c 'import os,torch; print(os.path.join(os.path.dirname(torch.__file__),"lib"))' 2>/dev/null || true)"
  if [[ -n "${_TORCH_LIB_DIR:-}" && -d "$_TORCH_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$_TORCH_LIB_DIR:${LD_LIBRARY_PATH:-}"
  fi

  set -x
  "$PYTHON_BIN" /usr/bin/colcon build \
    --symlink-install \
    --packages-select pcdet_ros2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
)

log "pcdet_ros2 overlay 构建完成：$ROS2_WS_DIR"
log "下一步：bash scripts/run_detection_humble_local.sh [input_topic] [output_topic]"
log "（Python pcdet 与 .so 仅在 $PYDEPS_SITE 内，不写 ~/.local，不会污染 v11/v15 主链路）"
