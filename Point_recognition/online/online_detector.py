import sys
from pathlib import Path
import numpy as np
import torch

# 将 DSVT 根目录加入环境变量，以便能正常导入 pcdet
dsvt_root = Path(__file__).resolve().parent.parent / "DSVT"
sys.path.append(str(dsvt_root))

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils

class OnlineDataset(DatasetTemplate):
    """
    用于在线推理的 Dataset 包装类
    """
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None):
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training,
            root_path=root_path, logger=logger
        )
        
    def prepare_single_data(self, points):
        """
        处理单帧点云数据
        points: (N, 4) numpy array，包含 [x, y, z, intensity]
        """
        input_dict = {
            'points': points,
            'frame_id': 0,
        }
        # prepare_data 会执行诸如点云范围过滤、体素化等操作
        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict

class OnlineDetector:
    """
    在线检测器：
    1. 初始化时加载模型和权重到 GPU，避免重复加载
    2. 提供 inference() 方法用于单帧实时推理
    """
    def __init__(self, cfg_file, ckpt_file):
        self.logger = common_utils.create_logger()
        
        # 1. 加载配置
        cfg_from_yaml_file(cfg_file, cfg)
        self.cfg = cfg
        
        # 2. 实例化在线 Dataset
        self.dataset = OnlineDataset(
            dataset_cfg=cfg.DATA_CONFIG, 
            class_names=cfg.CLASS_NAMES, 
            training=False,
            root_path=Path("/tmp"), # 占位路径，在线推理不从磁盘批量读数据
            logger=self.logger
        )
        
        # 3. 构建模型并加载权重
        self.model = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=self.dataset)
        self.model.load_params_from_file(filename=ckpt_file, logger=self.logger, to_cpu=True)
        self.model.cuda()
        self.model.eval()
        self.logger.info("Online Detector initialized successfully. Model is loaded in GPU.")

    def inference(self, points):
        """
        执行在线推理
        points: (N, 4) numpy array
        返回: boxes, scores, labels
        """
        # 数据预处理
        data_dict = self.dataset.prepare_single_data(points)
        
        # 组装为 batch 格式 (batch_size=1)
        data_dict = self.dataset.collate_batch([data_dict])
        load_data_to_gpu(data_dict)
        
        # 前向推理
        with torch.no_grad():
            pred_dicts, _ = self.model.forward(data_dict)
            
        pred_dict = pred_dicts[0]
        
        # 转成 numpy 格式返回
        boxes = pred_dict['pred_boxes'].cpu().numpy()
        scores = pred_dict['pred_scores'].cpu().numpy()
        labels = pred_dict['pred_labels'].cpu().numpy()
        
        return boxes, scores, labels
