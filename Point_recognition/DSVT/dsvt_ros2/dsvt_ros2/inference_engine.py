"""
DSVT/OpenPCDet & PointPillars 推理引擎封装。

支持两种模型格式:
1. OpenPCDet 模型 (DSVT, SECOND, PointPillars-OpenPCDet)
   - 需要 .yaml 配置 + .pth/.ckpt 权重
2. PointPillars 独立模型
   - 直接指定类别名和点云范围

Usage:
    # OpenPCDet 模型
    engine = InferenceEngine(cfg_file='config.yaml', ckpt_path='model.pth')

    # PointPillars 独立模型
    engine = PointPillarsEngine(
        ckpt_path='model.pth',
        class_names=['Soil'],
        point_cloud_range=[-75.2, -75.2, -2, 75.2, 75.2, 4],
    )

    boxes, scores, labels = engine.infer(points)  # points: (N, 4) numpy
"""

import numpy as np
import torch


class InferenceEngine:
    """通用 OpenPCDet 推理引擎。

    Parameters
    ----------
    cfg_file : str
        OpenPCDet 模型配置文件路径 (.yaml)
    ckpt_path : str
        模型权重文件路径 (.pth / .ckpt)
    device : str
        推理设备, 'cuda' 或 'cpu'
    score_thresh : float
        置信度阈值, 低于此值的检测结果将被过滤
    """

    def __init__(self, cfg_file, ckpt_path, device='cuda', score_thresh=0.1):
        self.score_thresh = score_thresh
        self.device = device

        from pcdet.config import cfg, cfg_from_yaml_file
        from pcdet.datasets import DatasetTemplate
        from pcdet.models import build_network, load_data_to_gpu
        from pcdet.utils import common_utils

        # 1. 加载配置
        cfg_from_yaml_file(cfg_file, cfg)
        self.cfg = cfg
        self.class_names = cfg.CLASS_NAMES

        # 2. Logger
        logger = common_utils.create_logger()

        # 3. 创建最小的 Dataset（仅用于数据预处理流水线）
        class _PreprocessDataset(DatasetTemplate):
            def __init__(s, dataset_cfg, class_names, training=False):
                super().__init__(
                    dataset_cfg=dataset_cfg,
                    class_names=class_names,
                    training=training,
                    root_path=None,
                    logger=logger,
                )

            def __len__(s):
                return 1

            def __getitem__(s, index):
                return None

        self.dataset = _PreprocessDataset(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=cfg.CLASS_NAMES,
            training=False,
        )

        # 4. 构建模型
        logger.info('Building model from: %s', cfg_file)
        self.model = build_network(
            model_cfg=cfg.MODEL,
            num_class=len(cfg.CLASS_NAMES),
            dataset=self.dataset,
        )
        self.model.load_params_from_file(
            filename=ckpt_path,
            logger=logger,
            to_cpu=(device == 'cpu'),
        )

        if device == 'cuda':
            self.model.cuda()
        self.model.eval()

        logger.info('Model loaded. Class names: %s', self.class_names)

    def infer(self, points):
        """对单帧点云执行推理。

        Parameters
        ----------
        points : np.ndarray
            原始点云, shape (N, 4+), 列顺序: [x, y, z, intensity, ...]

        Returns
        -------
        boxes : np.ndarray
            检测框, shape (M, 7), [x, y, z, dx, dy, dz, heading]
        scores : np.ndarray, shape (M,)
        labels : np.ndarray, shape (M,)
        """
        from pcdet.models import load_data_to_gpu

        input_dict = {
            'points': points.astype(np.float32),
            'frame_id': 0,
        }
        data_dict = self.dataset.prepare_data(data_dict=input_dict)
        data_dict = self.dataset.collate_batch([data_dict])
        load_data_to_gpu(data_dict)

        with torch.no_grad():
            pred_dicts, _ = self.model.forward(data_dict)

        preds = pred_dicts[0]
        boxes = preds['pred_boxes'].cpu().numpy()
        scores = preds['pred_scores'].cpu().numpy()
        labels = preds['pred_labels'].cpu().numpy()

        if self.score_thresh > 0:
            mask = scores >= self.score_thresh
            boxes, scores, labels = boxes[mask], scores[mask], labels[mask]

        return boxes, scores, labels

    def get_class_name(self, label_id):
        """将类别 ID 转为名称。"""
        if 0 <= label_id < len(self.class_names):
            return self.class_names[label_id]
        return f'class_{label_id}'


# ---------------------------------------------------------------------------
# Mock 工具
# ---------------------------------------------------------------------------

def _mock_unused_modules():
    """Mock PointPillars 推理中用不到的模块 (cv2, open3d 等)。

    这些模块仅在 PointPillars 的可视化工具中使用, 推理完全不需要。
    通过 mock 避免强制安装 open3d / opencv-python。
    """
    import sys
    import types

    # ---- mock open3d ----
    if 'open3d' not in sys.modules:
        o3d = types.ModuleType('open3d')
        # 子模块
        for sub in ['geometry', 'visualization', 'io', 'utility',
                     'camera', 'pipelines', 't']:
            sub_mod = types.ModuleType(f'open3d.{sub}')
            sys.modules[f'open3d.{sub}'] = sub_mod
            setattr(o3d, sub, sub_mod)
        sys.modules['open3d'] = o3d

    # ---- mock cv2 ----
    if 'cv2' not in sys.modules:
        cv2 = types.ModuleType('cv2')
        for attr in ['imread', 'imshow', 'waitKey', 'cvtColor', 'resize',
                      'imwrite', 'rectangle', 'putText', 'destroyAllWindows',
                      'VideoCapture', 'VideoWriter', 'COLOR_BGR2RGB',
                      'FONT_HERSHEY_SIMPLEX', 'COLOR_BGR2GRAY']:
            setattr(cv2, attr, lambda *a, **kw: None)
        sys.modules['cv2'] = cv2


# ---------------------------------------------------------------------------
# PointPillars 独立推理引擎
# ---------------------------------------------------------------------------


class PointPillarsEngine:
    """PointPillars 独立模型推理引擎。

    适用于 /home/libo/PointPillars/ 项目中训练的 PointPillars 模型。

    Parameters
    ----------
    ckpt_path : str
        PointPillars 模型权重文件路径 (.pth)
    class_names : list[str]
        类别名称列表, e.g. ['Soil']
    point_cloud_range : list[float]
        点云范围 [xmin, ymin, zmin, xmax, ymax, zmax]
    device : str
        推理设备
    score_thresh : float
        检测置信度阈值
    nms_thresh : float
        NMS IoU 阈值
    """

    def __init__(
        self,
        ckpt_path,
        class_names=None,
        point_cloud_range=None,
        device='cuda',
        score_thresh=0.1,
        nms_thresh=0.01,
    ):
        import sys

        # 把 PointPillars 项目加入路径
        pp_root = '/home/libo/PointPillars'
        if pp_root not in sys.path:
            sys.path.insert(0, pp_root)

        # ---- Mock 推理不需要的模块 ----
        _mock_unused_modules()

        from pointpillars.model import PointPillars

        self.device = device
        self.score_thresh = score_thresh
        self.class_names = class_names or ['Soil']
        self.nclasses = len(self.class_names)

        if point_cloud_range is None:
            point_cloud_range = [-75.2, -75.2, -2, 75.2, 75.2, 4]
        self.point_cloud_range = np.array(point_cloud_range, dtype=np.float32)

        # 构建模型
        anchor_ranges = [point_cloud_range] * self.nclasses
        self.model = PointPillars(
            nclasses=self.nclasses,
            point_cloud_range=point_cloud_range,
            anchor_ranges=anchor_ranges,
            anchor_sizes=PointPillars.DEFAULT_ANCHOR_SIZES[:self.nclasses],
        )

        # 加载权重
        ckpt = torch.load(ckpt_path, map_location='cpu')
        state = ckpt.get('model_state', ckpt)
        self.model.load_state_dict(state)

        # 调整阈值
        self.model.score_thr = score_thresh
        self.model.nms_thr = nms_thresh

        if device == 'cuda':
            self.model.cuda()
        self.model.eval()

    def infer(self, points):
        """对单帧点云执行推理。

        Parameters
        ----------
        points : np.ndarray
            原始点云, shape (N, 4+), [x, y, z, intensity, ...]

        Returns
        -------
        boxes : np.ndarray
            检测框, shape (M, 7), [x, y, z, dx, dy, dz, heading]
        scores : np.ndarray, shape (M,)
        labels : np.ndarray, shape (M,)
        """
        # 点云范围过滤
        pcr = self.point_cloud_range
        mask = (
            (points[:, 0] > pcr[0]) & (points[:, 0] < pcr[3])
            & (points[:, 1] > pcr[1]) & (points[:, 1] < pcr[4])
            & (points[:, 2] > pcr[2]) & (points[:, 2] < pcr[5])
        )
        points = points[mask]

        if len(points) == 0:
            return np.empty((0, 7)), np.empty((0,)), np.empty((0,))

        pc_tensor = torch.from_numpy(points.astype(np.float32))
        if self.device == 'cuda':
            pc_tensor = pc_tensor.cuda()

        with torch.no_grad():
            result = self.model(batched_pts=[pc_tensor], mode='test')[0]

        if isinstance(result, dict):
            boxes = result['lidar_bboxes']
            labels = result['labels']
            scores = result['scores']
        elif isinstance(result, (list, tuple)) and len(result) == 3:
            # 无检测时返回 ([], [], [])
            boxes, labels, scores = result
        else:
            boxes, labels, scores = [], [], []

        if len(boxes) == 0:
            return np.empty((0, 7)), np.empty((0,)), np.empty((0,))

        return (
            np.array(boxes, dtype=np.float32),
            np.array(scores, dtype=np.float32),
            np.array(labels, dtype=np.int32),
        )

    def get_class_name(self, label_id):
        """将类别 ID 转为名称。"""
        if 0 <= label_id < len(self.class_names):
            return self.class_names[label_id]
        return f'class_{label_id}'


# ---------------------------------------------------------------------------
# 聚类检测器引擎
# ---------------------------------------------------------------------------


class ClusterEngine:
    """无监督聚类检测器 (无需模型训练)。

    适用于没有标注数据的场景, 直接用欧氏聚类找物体。
    """

    def __init__(self, class_names=None, **kwargs):
        from dsvt_ros2.cluster_detector import ClusterDetector

        self.class_names = class_names or ['object']
        self.detector = ClusterDetector()  # uses defaults: eps=0.45, min_points=10

    def infer(self, points):
        return self.detector.detect(points)

    def get_class_name(self, label_id):
        return self.class_names[0] if self.class_names else 'object'


# ---------------------------------------------------------------------------
# 工厂函数
# ---------------------------------------------------------------------------


def create_engine(
    cfg_file=None,
    ckpt_path=None,
    engine_type='auto',
    class_names=None,
    point_cloud_range=None,
    device='cuda',
    score_thresh=0.1,
    **kwargs,
):
    """自动检测并创建合适的推理引擎。

    检测逻辑:
    - engine_type='cluster' → ClusterEngine (无监督)
    - engine_type='pointpillars' → PointPillarsEngine
    - engine_type='openpcdet' → InferenceEngine
    - engine_type='auto' → 自动检测
    """
    import os

    if engine_type == 'cluster':
        return ClusterEngine(class_names=class_names, **kwargs)

    if engine_type == 'pointpillars':
        return PointPillarsEngine(
            ckpt_path=ckpt_path,
            class_names=class_names or ['Soil'],
            point_cloud_range=point_cloud_range,
            device=device,
            score_thresh=score_thresh,
        )

    if engine_type == 'openpcdet' or cfg_file:
        if cfg_file and os.path.exists(cfg_file):
            return InferenceEngine(
                cfg_file=cfg_file,
                ckpt_path=ckpt_path,
                device=device,
                score_thresh=score_thresh,
            )

    # auto 模式
    if cfg_file and os.path.exists(cfg_file):
        return InferenceEngine(
            cfg_file=cfg_file,
            ckpt_path=ckpt_path,
            device=device,
            score_thresh=score_thresh,
        )

    if ckpt_path and os.path.exists(ckpt_path):
        return PointPillarsEngine(
            ckpt_path=ckpt_path,
            class_names=class_names or ['Soil'],
            point_cloud_range=point_cloud_range,
            device=device,
            score_thresh=score_thresh,
        )

    raise ValueError(
        'Cannot determine engine type. '
        'Provide --cfg_file for OpenPCDet models, --ckpt for PointPillars models, '
        'or use --engine_type cluster for unsupervised clustering.'
    )
