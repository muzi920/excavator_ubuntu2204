# Ceres Match Pipeline

本目录用于实现“原始点云土堆区域”和“挖掘机作业区域模板”之间的点云匹配链路。

当前目标：

1. 从原始 `pointcloud_base_link_*.pcd` 中提取 `0.02 <= z <= 0.5` 的土堆候选点。
2. 从 URDF 采样 bucket tip 的作业区域模板点云。
3. 通过配准估计模板到场景的刚体变换。
4. 输出匹配后的作业区域点云与最终可挖掘区域点云。

## 设计原则

- 不修改现有 `mode1` 稳定链路，全部新逻辑隔离在 `real_pcd/ceres_match/`。
- 先做 CPU 原型验证方法正确性，再接 Ceres 正式优化器。
- 高度带固定为 `0.02 <= z <= 0.5`。

## 规划中的文件

- `match_cpu_prototype.py`
  - 首版 Python CPU 原型
  - 基于 `numpy + scipy`
  - 输出 `json + pcd`

- `match_viz.py`
  - RViz 可视化：场景 / heap ROI / 匹配后模板 / 可挖区域

- `match_ceres.cc`
  - 后续 Ceres 正式版
  - 负责 SE(3) 参数优化

## 首版输入

- `--pcd`: 原始 `pointcloud_base_link_*.pcd`
- `--urdf`: 标定 URDF
- `--heap-z-min`: 默认 `0.02`
- `--heap-z-max`: 默认 `0.5`

## 首版输出

- `heap_roi_points.pcd`
- `workspace_template_points.pcd`
- `matched_workspace_points.pcd`
- `operable_region_points.pcd`
- `match_result.json`

## 验证顺序

1. 先看 `heap_roi_points.pcd`
2. 再看 `matched_workspace_points.pcd`
3. 最后看 `operable_region_points.pcd`

