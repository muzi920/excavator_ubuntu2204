#!/usr/bin/env python3
"""
DSVT / PointPillars 独立推理演示 (无需 ROS2)。

读取 .bin / .npy 点云文件 → GPU 推理 → Open3D 可视化。

支持两种模型:
  1. PointPillars 独立模型 (推荐, 已有训练好的 checkpoint)
  2. OpenPCDet 模型 (DSVT, SECOND 等)

用法:
    # PointPillars 模型 (已有 Soil checkpoint)
    python scripts/run_standalone_demo.py \
        --ckpt /home/libo/PointPillars/soil_logs/checkpoints/best.pth \
        --data_path /path/to/points.npy \
        --class_names Soil \
        --point_range -75.2,-75.2,-2,75.2,75.2,4

    # OpenPCDet 模型
    python scripts/run_standalone_demo.py \
        --cfg_file tools/cfgs/custom_models/second.yaml \
        --ckpt /path/to/checkpoint.pth \
        --data_path /path/to/points.bin

    # 批量推理 (不弹窗)
    python scripts/run_standalone_demo.py \
        --ckpt /path/to/checkpoint.pth \
        --data_path /path/to/pointcloud_dir \
        --no_vis --save_dir ./results

依赖: torch, numpy, open3d (可选, 用于可视化)
"""

import argparse
import glob
import os
import sys
import time
from pathlib import Path

import numpy as np

# 路径设置 (ROS2 包结构: dsvt_ros2/ → dsvt_ros2/ → *.py)
#   外层的 dsvt_ros2/ 是 ROS2 包根目录, 加入 sys.path 后
#   Python 能找到内层 dsvt_ros2/ 作为真正的 Python package
_SCRIPT_DIR = Path(__file__).resolve().parent           # .../dsvt_ros2/scripts/
_DSVT_ROS2_OUTER = _SCRIPT_DIR.parent                   # .../dsvt_ros2/ (ROS2 包根)
_PROJECT_ROOT = _DSVT_ROS2_OUTER.parent                  # .../DSVT/ (项目根)
sys.path.insert(0, str(_PROJECT_ROOT))                   # 用于 pcdet 导入
sys.path.insert(0, str(_DSVT_ROS2_OUTER))                # 用于 dsvt_ros2 导入

from dsvt_ros2.inference_engine import create_engine


def parse_args():
    parser = argparse.ArgumentParser(description='DSVT/PointPillars Standalone Inference Demo')

    # 模型参数 (二选一)
    model_group = parser.add_argument_group('Model')
    model_group.add_argument('--cfg_file', default=None,
                             help='OpenPCDet 模型配置文件 (.yaml)')
    model_group.add_argument('--ckpt', required=True,
                             help='模型权重文件 (.pth)')
    model_group.add_argument('--engine_type', default='auto',
                             choices=['auto', 'openpcdet', 'pointpillars'],
                             help='推理引擎类型 (默认 auto: 自动检测)')

    # PointPillars 专用参数
    pp_group = parser.add_argument_group('PointPillars options')
    pp_group.add_argument('--class_names', default='Soil',
                          help='类别名称, 逗号分隔 (默认: Soil)')
    pp_group.add_argument('--point_range', default='-75.2,-75.2,-2,75.2,75.2,4',
                          help='点云范围 xmin,ymin,zmin,xmax,ymax,zmax')

    # 数据参数
    data_group = parser.add_argument_group('Data')
    data_group.add_argument('--data_path', required=True,
                            help='点云文件路径 (.bin/.npy/.pcd) 或目录')
    data_group.add_argument('--ext', default='.npy',
                            help='data_path 为目录时, 点云文件扩展名 (默认: .npy)')

    # 推理参数
    infer_group = parser.add_argument_group('Inference')
    infer_group.add_argument('--score_thresh', type=float, default=0.1,
                             help='检测置信度阈值 (默认: 0.1)')
    infer_group.add_argument('--device', default='cuda',
                             help='推理设备 (cuda / cpu)')

    # 输出参数
    out_group = parser.add_argument_group('Output')
    out_group.add_argument('--no_vis', action='store_true',
                           help='禁用 3D 可视化')
    out_group.add_argument('--save_dir', default=None,
                           help='保存检测结果的目录 (txt 格式)')
    out_group.add_argument('--max_frames', type=int, default=None,
                           help='最大处理帧数')

    return parser.parse_args()


def load_point_cloud(file_path):
    """加载点云文件"""
    ext = Path(file_path).suffix.lower()
    if ext == '.bin':
        points = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
    elif ext == '.npy':
        points = np.load(file_path)
    elif ext == '.pcd':
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(file_path)
        xyz = np.asarray(pcd.points)
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            intensity = colors[:, 0]
            points = np.column_stack([xyz, intensity])
        else:
            points = np.column_stack([xyz, np.zeros((xyz.shape[0], 1))])
    else:
        raise ValueError(f'Unsupported: {ext}')
    return points


def visualize(points, boxes, scores, labels, class_names):
    """使用 Open3D 可视化点云和检测框"""
    try:
        from tools.visual_utils import open3d_vis_utils as V
    except ImportError:
        try:
            from visual_utils import open3d_vis_utils as V
        except ImportError:
            print('[WARN] Cannot import open3d_vis_utils, falling back to simple viz.')
            _simple_viz(points, boxes, scores, labels, class_names)
            return

    # 用 OpenPCDet 的可视化工具
    V.draw_scenes(
        points=points,
        ref_boxes=boxes,
        ref_scores=scores,
        ref_labels=labels,
    )


def _simple_viz(points, boxes, scores, labels, class_names):
    """简单 Open3D 可视化 (不依赖 OpenPCDet 工具)"""
    import open3d as o3d

    colors = [
        [0, 0, 1],    # blue
        [0, 1, 0],    # green
        [0, 1, 1],    # cyan
        [1, 1, 0],    # yellow
        [1, 0, 1],    # magenta
        [1, 0.5, 0],  # orange
    ]

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='DSVT Detection', width=1200, height=800)
    vis.get_render_option().point_size = 1.0
    vis.get_render_option().background_color = np.array([0.05, 0.05, 0.05])

    # 点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(np.ones((len(points), 3)) * 0.7)
    vis.add_geometry(pcd)

    # 坐标轴
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0)
    vis.add_geometry(axis)

    # 检测框
    for i in range(len(boxes)):
        x, y, z, dx, dy, dz, heading = boxes[i]
        label_id = int(labels[i])
        color = colors[label_id % len(colors)]

        R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, heading])
        box = o3d.geometry.OrientedBoundingBox([x, y, z], R, [dx, dy, dz])
        box.color = color
        vis.add_geometry(box)

    vis.run()
    vis.destroy_window()


def save_results(file_path, boxes, scores, labels, class_names):
    """保存检测结果为 KITTI-风格 txt 文件"""
    save_path = Path(file_path).with_suffix('.txt')
    with open(save_path, 'w') as f:
        for i in range(len(boxes)):
            x, y, z, dx, dy, dz, heading = boxes[i]
            cls_name = class_names[int(labels[i])] if int(labels[i]) < len(class_names) else 'unknown'
            # KITTI 格式: class x y z dx dy dz heading score
            f.write(f'{cls_name} {x:.4f} {y:.4f} {z:.4f} {dx:.4f} {dy:.4f} {dz:.4f} {heading:.4f} {scores[i]:.4f}\n')
    print(f'  Saved: {save_path}')


def main():
    args = parse_args()

    # ---- 收集文件列表 ----
    data_path = Path(args.data_path)
    if data_path.is_dir():
        file_list = sorted(glob.glob(str(data_path / f'*{args.ext}')))
    else:
        file_list = [str(data_path)]

    if not file_list:
        print(f'[ERROR] No files found in: {args.data_path}')
        sys.exit(1)

    if args.max_frames:
        file_list = file_list[:args.max_frames]

    print(f'Found {len(file_list)} files.')
    # ---- 解析 PointPillars 参数 ----
    class_names = [c.strip() for c in args.class_names.split(',')]
    point_cloud_range = [float(x) for x in args.point_range.split(',')]

    print(f'Config: {args.cfg_file or "(PointPillars standalone)"}')
    print(f'Checkpoint: {args.ckpt}')
    print(f'Classes: {class_names}')
    print(f'Score threshold: {args.score_thresh}')
    print()

    # ---- 加载模型 ----
    print('Loading model...')
    engine = create_engine(
        cfg_file=args.cfg_file,
        ckpt_path=args.ckpt,
        engine_type=args.engine_type,
        class_names=class_names,
        point_cloud_range=point_cloud_range,
        device=args.device,
        score_thresh=args.score_thresh,
    )
    print(f'Model loaded. Classes: {engine.class_names}')
    print()

    # ---- 创建保存目录 ----
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    # ---- 逐帧推理 ----
    total_time = 0.0
    for idx, fpath in enumerate(file_list):
        fname = Path(fpath).name
        print(f'[{idx+1}/{len(file_list)}] {fname} ', end='', flush=True)

        # 加载点云
        points = load_point_cloud(fpath)
        print(f'({points.shape[0]} pts) ', end='', flush=True)

        # 推理
        t0 = time.perf_counter()
        boxes, scores, labels = engine.infer(points)
        elapsed = (time.perf_counter() - t0) * 1000.0
        total_time += elapsed

        print(f'→ {len(boxes)} detections, {elapsed:.1f}ms')

        for i in range(min(len(boxes), 10)):  # 最多显示 10 个
            cls_name = engine.get_class_name(int(labels[i]))
            print(f'    [{cls_name}] score={scores[i]:.3f}, '
                  f'xyz=({boxes[i][0]:.2f},{boxes[i][1]:.2f},{boxes[i][2]:.2f})')

        # 保存
        if args.save_dir:
            save_path = os.path.join(args.save_dir, Path(fpath).name)
            save_results(save_path, boxes, scores, labels, engine.class_names)

        # 可视化
        if not args.no_vis:
            visualize(points, boxes, scores, labels, engine.class_names)

    # ---- 总结 ----
    avg_time = total_time / len(file_list)
    print(f'\n{"="*50}')
    print(f'Done. Processed {len(file_list)} frames.')
    print(f'Average latency: {avg_time:.1f}ms ({1000.0/avg_time:.1f} Hz)')
    print(f'Total time: {total_time:.1f}ms')


if __name__ == '__main__':
    main()
