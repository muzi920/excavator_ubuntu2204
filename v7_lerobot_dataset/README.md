# V7: LeRobot 数据集采集 (LeRobot Dataset)

本目录致力于为挖掘机控制任务采集符合端到端（End-to-End）强化学习或模仿学习标准的数据集。当前主要适配 Hugging Face 开源的 **LeRobot** 框架的数据格式。

## 主要工作与功能

1. **`lerobot_data_collector.py`**
   - **作用**：这是一个专门用于收集挖掘机传感器数据（如倾角、可能融合视觉）并将其格式化为 LeRobot 标准数据集的采集脚本。
   - **数据结构**：数据会被保存在 `data/excavator_dataset/` 目录下，包含 `meta/info.json` 元数据以及可能的分段轨迹（episodes）。
   - **使用场景**：在执行人为示教（如通过 GUI 遥控）的同时运行此脚本，记录各关节状态（observation）与动作指令（action），用于训练行为克隆（Behavior Cloning）等策略。

## 数据存储

- **`data/`**
  - 存放已采集的数据集，例如 `excavator_dataset`，内部包含轨迹记录文件与元数据（如 `meta/info.json`）。
