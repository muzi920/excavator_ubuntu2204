# DSVT 项目迁移说明与 Trae 上下文

这份文档用于项目目录迁移后的快速接管。新目录中的 Trae 可以先阅读本文，
再决定是否继续查看 `README.md`、`docs/INSTALL.md` 和
`docs/DATASET_CONVERSION_zh.md`。

## 项目定位

这是一个基于 OpenPCDet 体系扩展的 3D 点云检测项目，主体来源于
`DSVT: Dynamic Sparse Voxel Transformer`。仓库同时保留了官方 DSVT、
OpenPCDet 通用检测器、自定义数据集接入流程，以及一个 ROS 2 推理模块。

当前这个本地仓库除了原始 DSVT 用法，还承载了一套自定义单类点云检测流程：

- 原始标注数据位于 `kitti_data/`
- 转换后的自定义数据集位于 `data/custom/`
- 目标类别目前是 `Soil`
- 已新增中文使用手册，位于 `docs/DATASET_CONVERSION_zh.md`

## 关键目录

迁移后优先关注以下目录。

- `pcdet/`
  - 核心数据集、模型、算子、评估逻辑
- `tools/`
  - 训练、测试、demo、数据处理脚本入口
- `tools/cfgs/`
  - 所有训练与数据集配置文件
- `tools/process_tools/`
  - 数据转换与预处理脚本
- `data/custom/`
  - 当前已经生成好的 CustomDataset 数据
- `kitti_data/`
  - 原始 `pcd + kitti_labels` 数据
- `docs/`
  - 安装文档与中文流程说明
- `dsvt_ros2/`
  - ROS 2 推理相关代码

## 当前主要入口

如果 Trae 需要快速定位入口，优先看这些文件。

- `tools/train.py`
  - 训练入口
- `tools/test.py`
  - 评估入口
- `tools/demo.py`
  - 单文件或目录点云推理可视化入口
- `tools/process_tools/convert_kitti_data_to_custom.py`
  - 本地新增的数据转换脚本
- `pcdet/datasets/custom/custom_dataset.py`
  - 自定义数据集定义与 `custom_infos` 生成逻辑
- `tools/cfgs/custom_models/second.yaml`
  - 当前用于 `Soil` 单类训练的模型配置
- `tools/cfgs/dataset_configs/custom_dataset.yaml`
  - CustomDataset 的数据集配置

## 当前数据状态

仓库中已经同时存在原始数据和转换后的数据。

### 原始数据

原始数据位于 `kitti_data/`，目录结构如下：

```text
kitti_data/
  kitti_labels/*.txt
  point_clouds/default/*.pcd
```

特点如下：

- 点云是 `.pcd`
- 标签是 KITTI 风格文本
- 这批数据不是标准 KITTI 训练目录结构
- 训练时不直接使用这份原始目录

### 转换后数据

转换后的 CustomDataset 位于 `data/custom/`，目录结构如下：

```text
data/custom/
  points/*.npy
  labels/*.txt
  ImageSets/train.txt
  ImageSets/val.txt
  custom_infos_train.pkl
  custom_infos_val.pkl
  custom_dbinfos_train.pkl
  gt_database/
```

当前已知状态：

- 原始数据共 `469` 个样本
- 已转换为 `469` 个 `npy + txt`
- 当前划分为 `375` 个训练样本、`94` 个验证样本
- `gt_database` 已经生成过一次

## 当前自定义流程

本地使用的流程不是官方 Waymo/NuScenes 训练流程，而是下面这条自定义链路：

1. 将 `kitti_data/point_clouds/default/*.pcd` 转换为 `data/custom/points/*.npy`
2. 将 `kitti_data/kitti_labels/*.txt` 转换为 `data/custom/labels/*.txt`
3. 生成 `ImageSets/train.txt` 和 `ImageSets/val.txt`
4. 基于 `CustomDataset` 生成 `custom_infos_train.pkl`
5. 生成 `custom_dbinfos_train.pkl` 和 `gt_database/`
6. 使用 `tools/cfgs/custom_models/second.yaml` 启动训练

这条流程的详细中文说明已经写在：

- `docs/DATASET_CONVERSION_zh.md`

## 本地新增脚本

### `tools/process_tools/convert_kitti_data_to_custom.py`

这是本地新增脚本，不是原始 DSVT 官方仓库自带能力。

它负责：

- 读取 `pcd`
- 支持 `ascii` 和 `binary` PCD
- 提取 `x y z`
- 如果没有 `intensity`，自动补一个伪强度列
- 把 KITTI 风格标签的末尾 `h w l x y z ry` 重排为
  `x y z l w h angle`
- 输出为 `CustomDataset` 需要的格式

该脚本的重要假设：

- 它假定标签中的 3D 框已经在点云或 lidar 坐标系
- 它不会做相机坐标到 lidar 坐标的标定变换

如果迁移后要继续处理新数据，优先复用这个脚本。

## 当前配置状态

这里是迁移后最容易踩坑的部分。

### `tools/cfgs/custom_models/second.yaml`

这个文件当前已经明显偏向本地自定义任务：

- `CLASS_NAMES` 目前是 `['Soil']`
- `DENSE_HEAD` 的 anchor 类别名也是 `Soil`
- `_BASE_CONFIG_` 当前写成：

```yaml
DATA_CONFIG:
    _BASE_CONFIG_: tools/cfgs/dataset_configs/custom_dataset.yaml
```

这说明训练命令默认是从**仓库根目录**执行的。

### `tools/cfgs/dataset_configs/custom_dataset.yaml`

这个文件当前磁盘上的内容仍然保留了默认多类模板：

- `Vehicle`
- `Pedestrian`
- `Cyclist`

也就是说，当前仓库存在一个实际不一致状态：

- `second.yaml` 已经切到 `Soil`
- `custom_dataset.yaml` 仍然是默认三类模板

如果迁移后要继续训练 `Soil` 单类任务，Trae 需要优先检查并统一这两个文件。

建议统一项包括：

- `MAP_CLASS_TO_KITTI`
- `filter_by_min_points`
- `SAMPLE_GROUPS`
- `CLASS_NAMES`

## 当前代码层面的注意事项

以下是迁移后需要优先确认的几个代码事实。

### `pcdet/config.py`

当前 `merge_new_config()` 中读取 `_BASE_CONFIG_` 的方式是直接：

```python
with open(new_config['_BASE_CONFIG_'], 'r') as f:
```

这意味着：

- `_BASE_CONFIG_` 是按当前工作目录解析
- 不是按当前 YAML 文件所在目录解析

所以迁移后如果启动目录变化，配置加载很容易报路径错误。当前
`second.yaml` 使用仓库根目录相对路径，是为了适配这个现状。

### `pcdet/datasets/custom/custom_dataset.py`

这个文件是当前自定义流程的核心，但有两个重要点：

1. `__main__` 入口里 `create_custom_infos` 的类别被写死为：

```python
['Vehicle', 'Pedestrian', 'Cyclist']
```

所以如果要生成 `Soil` 的 `custom_infos`，不要直接使用该文件自带的默认
命令行入口，而是用 Python 片段显式传 `class_names=['Soil']`。

2. 当前文件里 `create_groundtruth_database()` 使用了 `Path(self.root_path)`，
   但文件顶部没有看到 `from pathlib import Path`。

这表示迁移后如果重新跑数据准备，这里可能再次触发 `NameError`。Trae 在接手
这个仓库时，需要先核对这个导入是否已经补上。

### `pcdet/models/backbones_3d/dsvt.py`

当前 `TensorRT` 的导入在 `DSVT_TrtEngine` 内部才执行，而不是在模块顶层执行。

这意味着：

- 不使用 TensorRT 路线时，普通训练和推理不必依赖 `tensorrt` 包
- 如果要走 TensorRT 部署路线，再单独检查 `trt` 环境即可

## 文档现状

仓库里当前可用的文档包括：

- `README.md`
  - 官方项目说明，偏 DSVT 原始能力
- `docs/INSTALL.md`
  - 环境安装说明
- `docs/DATASET_CONVERSION_zh.md`
  - 当前本地自定义数据集流程的中文手册

其中最贴近当前实际使用方式的是：

- `docs/DATASET_CONVERSION_zh.md`

## 环境与依赖

这个项目本质上仍然是 OpenPCDet/DSVT 体系，迁移后通常需要重新确认：

- Python 环境
- PyTorch 与 CUDA 版本
- `spconv`
- 编译型算子是否可正常导入
- `python setup.py develop` 是否已经执行

安装说明主要参考：

- `docs/INSTALL.md`

## 训练与评估的当前约定

迁移后如果 Trae 要排查训练问题，先假定使用如下约定。

- 训练命令从仓库根目录启动
- 当前自定义任务走 `tools/cfgs/custom_models/second.yaml`
- 数据目录默认是 `data/custom`
- demo 默认读取 `.bin` 或 `.npy`，不直接读取 `.pcd`

常见入口命令如下：

```bash
python tools/train.py --cfg_file tools/cfgs/custom_models/second.yaml
python tools/test.py --cfg_file tools/cfgs/custom_models/second.yaml --ckpt <ckpt>
python tools/demo.py --cfg_file tools/cfgs/custom_models/second.yaml --data_path data/custom/points --ext .npy
```

## 当前实验结果背景

本地已经出现过一组 `Soil` 单类评估结果，数值大致如下：

```text
3D AP   79.25%
BEV AP  94.58%
Bbox AP 97.75%
AOS     97.70
```

这说明当前自定义数据链路至少已经跑通过一次训练和评估流程，但迁移后不要默认
所有配置都已经完全一致，因为磁盘上的 YAML 仍存在前述不一致状态。

## ROS 2 模块

仓库还带有一个独立的 ROS 2 包：

- `dsvt_ros2/`

这个目录与当前自定义 `Soil` 数据训练流程不是强绑定关系，但如果迁移目标包含
在线推理、节点封装或可视化，这部分也需要一并迁移。

## 迁移建议

建议迁移时至少保留以下内容：

```text
DSVT/
  pcdet/
  tools/
  docs/
  data/custom/
  kitti_data/                # 如果还要重新转换或回溯原始数据
  dsvt_ros2/                 # 如果后续要接 ROS 2
  TRAE_PROJECT_CONTEXT.md
```

如果只关心训练和评估，最少需要保留：

- `pcdet/`
- `tools/`
- `tools/cfgs/`
- `data/custom/`
- `docs/DATASET_CONVERSION_zh.md`
- `TRAE_PROJECT_CONTEXT.md`

## Trae 接手建议

新目录中的 Trae 在开始任何修改前，建议按以下顺序检查。

1. 阅读 `TRAE_PROJECT_CONTEXT.md`
2. 阅读 `docs/DATASET_CONVERSION_zh.md`
3. 检查 `tools/cfgs/custom_models/second.yaml`
4. 检查 `tools/cfgs/dataset_configs/custom_dataset.yaml`
5. 检查 `pcdet/config.py` 的 `_BASE_CONFIG_` 路径解析逻辑
6. 检查 `pcdet/datasets/custom/custom_dataset.py` 的 `Path` 导入和
   `class_names` 硬编码

如果需要继续维护 `Soil` 单类训练链路，最优先事项不是改模型，而是先统一：

- 数据集类别定义
- YAML 配置
- `custom_infos` 生成方式
- 启动目录和配置路径解析

## 一句话总结

这是一个以 DSVT/OpenPCDet 为底座、已经接入本地 `Soil` 单类点云数据的仓库。
当前最重要的不是重新理解 DSVT 论文，而是保住并理顺这条本地自定义数据转换、
配置、训练、评估链路。
