# DSVT (Dynamic Sparse Voxel Transformer)

DSVT 是一个用于 3D 点云目标检测的高效 Transformer 架构。本工程提供了从自定义数据集（如 `kitti_data`）的转换、模型训练、评估到推理可视化的完整工作流。

## 一、环境依赖

- Linux (推荐 Ubuntu 18.04/20.04/22.04)
- Python 3.6+
- PyTorch 1.1+ (推荐 1.5 ~ 1.10)
- CUDA 9.0+ (PyTorch 1.3+ 需要 CUDA 9.2+)
- `spconv` (推荐使用官方 `v2.x` 版本)
- `torch-scatter`

**安装步骤**：
在项目根目录执行以下命令安装核心依赖：
```bash
python setup.py develop
```
*(注：详细环境安装可参考 `docs/INSTALL.md`)*

---

## 二、数据集准备与转换

如果你使用自己的标注数据（格式类似 KITTI），需要先将其转换为 DSVT 支持的 `CustomDataset` 格式。

### 1. 目录结构准备
将你的点云（`.pcd` 或 `.bin`）和标签（`.txt`）放入 `kitti_data/` 目录：
```text
kitti_data/
  kitti_labels/0000.txt
  point_clouds/default/0000.pcd
```

### 2. 执行转换脚本
使用提供的转换工具将数据整理到 `data/custom/` 目录，这里以 `Soil` 类别为例：
```bash
python tools/process_tools/convert_kitti_data_to_custom.py \
  --src-root kitti_data \
  --dst-root data/custom \
  --classes Soil \
  --assume-lidar-labels \
  --shuffle
```
转换完成后，会在 `data/custom/` 下生成 `.npy` 点云文件、新格式的 `.txt` 标签以及 `ImageSets/train.txt` 等划分文件。

### 3. 生成数据索引 (Infos)
为加速训练，需要提取数据集信息并建立 ground truth 数据库：
```bash
python - <<'PY'
import yaml
from easydict import EasyDict
from pathlib import Path
from pcdet.datasets.custom.custom_dataset import create_custom_infos

cfg = EasyDict(yaml.safe_load(open('tools/cfgs/dataset_configs/custom_dataset.yaml')))
create_custom_infos(dataset_cfg=cfg, class_names=['Soil'],
                    data_path=Path('data/custom'), save_path=Path('data/custom'), workers=4)
PY
```

---

## 三、模型训练

### 1. 配置参数
训练前请检查 `tools/cfgs/custom_models/second.yaml`（或你选用的网络配置文件）：
- `CLASS_NAMES`: 确保与你要训练的类别（如 `['Soil']`）一致。
- `ANCHOR_GENERATOR_CONFIG`: 确保 Anchor 尺寸（`anchor_sizes`）符合实际物体的平均尺寸。

### 2. 启动训练
运行以下命令开始训练（默认 80 个 Epoch）：
```bash
python tools/train.py --cfg_file tools/cfgs/custom_models/second.yaml
```
- 模型权重 (`.pth`) 和日志将保存在 `output/cfgs/custom_models/second/default/` 目录下。
- 你可以使用 `--batch_size` 或 `--extra_tag` 参数自定义批次和实验名称。

---

## 四、模型评估

使用验证集（`val.txt`）评估某个具体权重文件的精度（如 `AP`、`AOS` 等）：
```bash
python tools/test.py \
  --cfg_file tools/cfgs/custom_models/second.yaml \
  --ckpt output/cfgs/custom_models/second/default/ckpt/checkpoint_epoch_79.pth
```
如果你想在训练期间自动监控并评估所有新生成的权重，可加上 `--eval_all` 参数。

---

## 五、如何使用权重进行推理与可视化

你可以使用 `demo.py` 调用训练好的权重 (`.pth`) 对单帧或批量点云进行 3D 目标检测预测，并实时可视化。

*(注意：可视化需要安装 `open3d` 或 `mayavi`)*

### 1. 查看模型预测结果（单帧）
```bash
python tools/demo.py \
  --cfg_file tools/cfgs/custom_models/second.yaml \
  --ckpt output/cfgs/custom_models/second/default/ckpt/checkpoint_epoch_79.pth \
  --data_path data/custom/points/0005.npy \
  --ext .npy
```
在弹出的 3D 窗口中，**绿色的 3D 框**即为模型预测出的目标位置。

### 2. 对比预测框与真实标注框 (Ground Truth)
如果你想看看模型预测得准不准，可以附加上真实标签目录，此时屏幕会同时渲染预测框和标注框：
```bash
python tools/demo.py \
  --cfg_file tools/cfgs/custom_models/second.yaml \
  --ckpt output/cfgs/custom_models/second/default/ckpt/checkpoint_epoch_79.pth \
  --data_path data/custom/points/0005.npy \
  --ext .npy \
  --labels_dir data/custom/labels
```

### 3. 批量浏览整个目录
将 `--data_path` 指向一个目录，关闭当前 3D 窗口后，会自动弹出下一帧点云的预测结果：
```bash
python tools/demo.py \
  --cfg_file tools/cfgs/custom_models/second.yaml \
  --ckpt output/cfgs/custom_models/second/default/ckpt/checkpoint_epoch_79.pth \
  --data_path data/custom/points \
  --ext .npy
```

---

## 六、常见问题 (FAQ)

### 1. 编译 pcdet (DSVT) 时遇到 CUDA 版本不匹配报错

**报错信息**：
```text
RuntimeError: 
The detected CUDA version (13.1) mismatches the version that was used to compile
PyTorch (12.8). Please make sure to use the same CUDA versions.
```

**原因分析**：
这个错误是由 PyTorch 的 C++ 扩展编译脚本（`cpp_extension.py`）主动抛出的。它的意思是：当前系统默认被激活的 CUDA 编译器版本（`nvcc`，如版本 13.1）与当前 Python 环境中安装的 PyTorch 所依赖的 CUDA 版本（如 12.8）差距过大。出于安全考虑，PyTorch 拒绝编译底层的自定义 CUDA 算子（如 `pcdet`）。

**解决步骤**：

核心思路是：**不改变系统默认的全局配置，而是利用 PyTorch 的编译环境变量 `CUDA_HOME` 和 `TORCH_CUDA_ARCH_LIST`，强制让编译脚本使用指定路径下（与 PyTorch 更兼容）的 CUDA 编译器。**

1. **排查系统现有的 CUDA 版本**：
   查看系统目录 `/usr/local/` 下安装了哪些版本的 CUDA：
   ```bash
   ls -l /usr/local/ | grep cuda
   ```
   例如，发现除了软链接指向的 `cuda-13.1` 之外，还安装了 `cuda-12.9`。

2. **对齐 CUDA 环境变量 (`CUDA_HOME`)**：
   PyTorch 的编译脚本有一个关键逻辑：如果环境变量中设置了 `CUDA_HOME`，它会优先使用该路径作为编译器，而不是使用系统默认的 `/usr/local/cuda`。
   由于 12.9 和 PyTorch 要求的 12.8 属于同一个大版本（CUDA 12.x），PyTorch 会将其视为“小版本不匹配 (Minor version mismatch)”，从而降级为警告 (Warning) 而不是直接报错崩溃。
   ```bash
   export CUDA_HOME=/usr/local/cuda-12.9
   ```

3. **指定目标显卡的架构代号 (`TORCH_CUDA_ARCH_LIST`)**：
   如果不指定显卡架构，编译器会尝试编译所有它支持的架构，这往往会导致兼容性报错（如 `IndexError: list index out of range`）。通过 `nvidia-smi` 确认显卡型号后（例如 RTX 5070 等高算力显卡），强制指定几个主流且兼容性好的算力代号（如 Ampere 8.0, 8.6 和 Ada 8.9）：
   ```bash
   export TORCH_CUDA_ARCH_LIST="8.0;8.6;8.9"
   ```

4. **重新执行编译**：
   将上述环境变量组合起来临时注入，让 `setup.py` 或 `colcon build` 使用指定的环境成功编译：
   ```bash
   export CUDA_HOME=/usr/local/cuda-12.9
   export TORCH_CUDA_ARCH_LIST="8.0;8.6;8.9"
   
   # 如果使用 colcon build
   colcon build --packages-select pcdet
   
   # 如果直接使用 python setup.py
   python3 setup.py develop
   ```
