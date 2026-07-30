# 数据集转换与训练评估使用手册

以单类别 `Soil` 为例，覆盖从原始数据到训练评估的完整流程。

## 目录

- [一、数据集转换](#一数据集转换)
- [二、配置准备](#二配置准备)
- [三、训练](#三训练)
- [四、评估](#四评估)
- [五、可视化推理](#五可视化推理)
- [六、后处理参数调优](#六后处理参数调优)
- [七、常见问题](#七常见问题)

---

## 一、数据集转换

将原始 KITTI 格式数据转为 `CustomDataset` 格式。

```text
源目录:                           目标目录:
kitti_data/                       data/custom/
  kitti_labels/*.txt    →           points/*.npy     (N×4)
  point_clouds/default/*.pcd        labels/*.txt     (x y z l w h angle class)
                                    ImageSets/
                                      train.txt
                                      val.txt
```

<!-- prettier-ignore -->
> [!IMPORTANT]
> 脚本假定标签已处于点云/lidar 坐标系，不会自动完成相机→lidar 坐标变换。

原始标签（KITTI 风格）→ 目标标签（CustomDataset）的字段映射：

```text
class truncation ... h w l x y z ry   →   x y z l w h angle class
```

### 执行转换

```bash
python tools/process_tools/convert_kitti_data_to_custom.py \
  --src-root kitti_data \
  --dst-root data/custom \
  --classes Soil \
  --assume-lidar-labels \
  --shuffle
```

成功输出（469 样本 → 375 train / 94 val）：

```text
Converted files: 469  Train samples: 375  Val samples: 94  Classes: ['Soil']
```

| 常用参数 | 说明 |
|---|---|
| `--classes` | 保留的类别名 |
| `--train-ratio 0.8` | 训练集占比（默认 0.8） |
| `--shuffle` | 划分前打乱样本顺序 |
| `--seed 42` | 随机种子 |
| `--limit 10` | 只转换前 N 个样本，方便调试 |
| `--fake-intensity 0.0` | PCD 无强度字段时的默认值 |
| `--overwrite` | 覆盖已有输出 |

### 转换后自检

1. `points/` 和 `labels/` 一一对应
2. `ImageSets/train.txt` 和 `val.txt` 已生成
3. 标签每行为 `x y z l w h angle Soil`，`.npy` shape 为 `N×4`

---

## 二、配置准备

### 2.1 数据集配置（`tools/cfgs/dataset_configs/custom_dataset.yaml`）

只需确认以下关键字段与你的数据一致：

```yaml
DATA_PATH: 'data/custom'          # 必须相对于项目根目录

POINT_CLOUD_RANGE: [-75.2, -75.2, -2, 75.2, 75.2, 4]

MAP_CLASS_TO_KITTI: { 'Soil': 'Car' }   # 评估时映射为 KITTI 类别

DATA_SPLIT: { 'train': train, 'test': val }

INFO_PATH: { 'train': [custom_infos_train.pkl], 'test': [custom_infos_val.pkl] }
```

数据增强和体素化参数通常无需改动，如需调整参考原文件注释。

### 2.2 模型配置（`tools/cfgs/custom_models/second.yaml`）

```yaml
CLASS_NAMES: ['Soil']

DATA_CONFIG:
    _BASE_CONFIG_: ../dataset_configs/custom_dataset.yaml

MODEL:
    DENSE_HEAD:
        ANCHOR_GENERATOR_CONFIG: [
            { 'class_name': 'Soil',
              'anchor_sizes': [[3.9, 1.6, 1.56]], ... }
        ]

POST_PROCESSING:
    SCORE_THRESH: 0.3
    NMS_CONFIG: { NMS_THRESH: 0.1 }
```

<!-- prettier-ignore -->
> [!NOTE]
> anchor 尺寸为初始值，如果 `Soil` 实际尺寸与车辆差异大，建议根据标注框统计调整。

### 2.3 生成索引文件

内建入口的类别硬编码为 `['Vehicle', 'Pedestrian', 'Cyclist']`，请改用以下脚本：

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

生成文件及用途：

| 文件 | 用途 |
|---|---|
| `custom_infos_train.pkl` | 训练集索引（375 样本） |
| `custom_infos_val.pkl` | 验证集索引（94 样本） |
| `custom_dbinfos_train.pkl` | GT 采样增强数据库 |
| `gt_database/` | 每个标注目标的裁剪点云 |

成功输出：`Database Soil: 375` → `Data preparation done`

### 2.4 启动前检查

- `data/custom/` 下存在上述 4 项文件/目录
- 两个 YAML 的类别名都是 `Soil`，`_BASE_CONFIG_` 路径正确
- 环境已安装 `torch`、`spconv`、`yaml`、`easydict`

---

## 三、训练

### 3.1 基本命令

```bash
python tools/train.py --cfg_file tools/cfgs/custom_models/second.yaml
```

默认训练 80 个 epoch，每 epoch 保存 checkpoint，训练结束后自动评估。

### 3.2 常用参数

```bash
python tools/train.py --cfg_file tools/cfgs/custom_models/second.yaml \
    --batch_size 8 \
    --extra_tag exp_v2 \
    --fix_random_seed \
    --set OPTIMIZATION.NUM_EPOCHS 5   # 快速验证 5 个 epoch
```

| 参数 | 说明 |
|---|---|
| `--batch_size` | 覆盖 YAML 中的 `BATCH_SIZE_PER_GPU` |
| `--extra_tag` | 实验标签（默认 `default`），不同实验互不覆盖 |
| `--ckpt <path>` | 断点恢复（自动恢复优化器状态） |
| `--pretrained_model <path>` | 加载预训练权重 |
| `--fix_random_seed` | 固定随机种子 666 |
| `--fp16` | 混合精度训练 |
| `--set K.V` | 覆盖任意配置项 |

### 3.3 输出结构

```text
output/cfgs/custom_models/second/default/
  ├── ckpt/checkpoint_epoch_N.pth
  ├── eval/epoch_N/val/default/       # 每轮评估结果
  ├── log_train_YYYYMMDD-HHMMSS.txt
  ├── second.yaml                     # 配置副本
  └── tensorboard/
```

### 3.4 日志解读

```text
epoch: 78/80, cur_iter=67/94, loss=0.1931,
time_cost(epoch): 00:17/00:06, time_cost(all): 30:36/00:30,
d_time=0.15 f_time=0.09 b_time=0.24 norm=2.0716 lr=5.35e-06
```

- `d_time/f_time/b_time`：数据加载/前向/反向耗时（括号内为滑动平均）
- `norm`：梯度范数
- `lr`：当前学习率（one-cycle：先升后降）

健康信号：loss 持续下降并趋于平稳（~5 → ~0.15），`norm` 不剧烈震荡。

---

## 四、评估

评估数据：`val.txt` 中的 94 个样本（与训练集不相交），不做数据增强。

### 4.1 基本命令

```bash
# 单 checkpoint
python tools/test.py --cfg_file tools/cfgs/custom_models/second.yaml \
    --ckpt output/cfgs/custom_models/second/default/ckpt/checkpoint_epoch_79.pth

# 自动评估所有未评估过的 checkpoint
python tools/test.py --cfg_file tools/cfgs/custom_models/second.yaml --eval_all
```

| 参数 | 说明 |
|---|---|
| `--ckpt` | checkpoint 路径 |
| `--extra_tag` | 实验标签，默认 `default` |
| `--eval_all` | 持续监控并评估所有新 checkpoint |
| `--save_to_file` | 保存详细结果 |

### 4.2 结果解读

`Soil` 通过 `MAP_CLASS_TO_KITTI` 映射为 `Car` 输出：

```text
┌─────────┬─────────┬─────────────┐
│  指标   │ AP@0.70 │ AP_R40@0.70 │
├─────────┼─────────┼─────────────┤
│ 3D AP   │  XX.X%  │  XX.X%      │
├─────────┼─────────┼─────────────┤
│ BEV AP  │  XX.X%  │  XX.X%      │
├─────────┼─────────┼─────────────┤
│ Bbox AP │  XX.X%  │  XX.X%      │
├─────────┼─────────┼─────────────┤
│ AOS     │  XX.X   │  XX.X       │
└─────────┴─────────┴─────────────┘
recall_rcnn_0.3: 1.000    Average predicted objects: N
```

| 指标 | 含义 |
|---|---|
| **3D AP** | 三维检测精度（核心指标） |
| **BEV AP** | 鸟瞰图视角精度 |
| **AOS** | 朝向估计质量 |
| **AP / AP_R40** | 11 点插值 / 40 点召回插值 |

`recall_rcnn` ≈ 1.0 说明漏检少；`Average predicted objects` 远大于标注数说明假阳性多，需提高 `SCORE_THRESH`。

---

## 五、可视化推理

需要 `pip install open3d`（优先）或 Mayavi。

```bash
# 单帧
python tools/demo.py --cfg_file tools/cfgs/custom_models/second.yaml \
    --ckpt output/.../checkpoint_epoch_79.pth \
    --data_path data/custom/points/0005.npy --ext .npy

# 整个目录
python tools/demo.py --cfg_file tools/cfgs/custom_models/second.yaml \
    --ckpt output/.../checkpoint_epoch_79.pth \
    --data_path data/custom/points --ext .npy
```

# 看预测 + 真实框
python tools/demo.py --cfg_file tools/cfgs/custom_models/second.yaml \
    --ckpt output/cfgs/custom_models/second/default/ckpt/checkpoint_epoch_79.pth \
    --data_path data/custom/points/0005.npy --ext .npy \
    --labels_dir data/custom/labels

# 只看预测框（不加 --labels_dir）
python tools/demo.py --cfg_file tools/cfgs/custom_models/second.yaml \
    --ckpt output/cfgs/custom_models/second/default/ckpt/checkpoint_epoch_79.pth \
    --data_path data/custom/points/0005.npy --ext .npy

| 参数 | 说明 |
|---|---|
| `--data_path` | 单文件 = 看一帧；目录 = 逐帧浏览 |
| `--ext` | `.npy` 或 `.bin`（默认 `.bin`） |

窗口中：绿色框 = 预测框，坐标轴红=X 绿=Y 蓝=Z。目录模式关闭窗口自动弹出下一帧。

---

## 六、后处理参数调优

以下参数在 `second.yaml` 的 `POST_PROCESSING` 段，修改后无需重新训练：

| 参数 | 默认值 | 作用 | 建议 |
|---|---|---|---|
| `SCORE_THRESH` | 0.1 | 低于该置信度的框丢弃 | 从 0.3 起调，假阳性多则提高 |
| `NMS_THRESH` | 0.01 | IoU 超阈值则去重 | 从 0.1 起调，重复框多则降低 |

调优流程：评估 → 看 `Average predicted number` → 按需调整 → 重新评估确认 3D AP 未下降。

---

## 七、常见问题

**Q1: `BACKUP_DB_INFO` 报错** → `DATA_PATH` 写成了 `'../data/custom'`，改为 `'data/custom'`。

**Q2: torch.load UnpicklingError** → PyTorch 2.6 默认 `weights_only=True`，已在以下文件添加 `weights_only=False`：`detector3d_template.py`、`sem_deeplabv3.py`、`ddn_template.py`。

**Q3: 评估阶段 numba 报 Signature mismatch** → numba 0.66 CUDA JIT 兼容问题，`rotate_iou.py` 已替换为 CPU 实现。

**Q4: CUDA OOM** → 减小 `BATCH_SIZE_PER_GPU` / `MAX_NUMBER_OF_VOXELS`，或增大 `VOXEL_SIZE`。

**Q5: 3D AP 为 0** → 检查 `MAP_CLASS_TO_KITTI` 映射、anchor 尺寸、`POINT_CLOUD_RANGE`。

**Q6: 可视化黑屏** → 确认 `--ext` 和 `--data_path` 正确，用滚轮缩放/右键拖拽调整视角。

**Q7: 转换失败** → 确认 PCD 与标签一一对应、`--classes` 名称一致、标签在 lidar 坐标系下。
