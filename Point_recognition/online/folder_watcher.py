import time
import os
import argparse
from pathlib import Path
import numpy as np

from online_detector import OnlineDetector

def watch_and_infer(detector, watch_dir, ext='.npy'):
    """
    监控指定目录，如果有新的点云文件写入，则进行在线推理
    """
    watch_dir = Path(watch_dir)
    if not watch_dir.exists():
        watch_dir.mkdir(parents=True)
        
    print(f"Start watching directory: {watch_dir} for {ext} files...")
    
    processed_files = set()
    
    try:
        while True:
            # 获取当前目录下的所有点云文件
            current_files = set(watch_dir.glob(f"*{ext}"))
            new_files = current_files - processed_files
            
            # 排序确保按顺序处理
            for file_path in sorted(new_files):
                print(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] Processing new file: {file_path.name}")
                
                # 读取点云
                if ext == '.npy':
                    points = np.load(str(file_path))
                elif ext == '.bin':
                    points = np.fromfile(str(file_path), dtype=np.float32).reshape(-1, 4)
                else:
                    raise NotImplementedError(f"Extension {ext} not supported")
                
                # 确保维度为 Nx4
                if points.shape[1] < 4:
                    padding = np.zeros((points.shape[0], 4 - points.shape[1]), dtype=points.dtype)
                    points = np.hstack([points, padding])
                elif points.shape[1] > 4:
                    points = points[:, :4]
                
                # 执行在线推理
                start_time = time.time()
                boxes, scores, labels = detector.inference(points)
                cost_time = time.time() - start_time
                
                print(f"  -> Inference done in {cost_time*1000:.2f} ms")
                print(f"  -> Detected {len(boxes)} objects.")
                for i in range(len(boxes)):
                    print(f"     Obj {i+1}: Label={labels[i]}, Score={scores[i]:.4f}, Box={boxes[i]}")
                
                processed_files.add(file_path)
                
            # 短暂休眠，降低 CPU 占用
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nStop watching.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Online Point Cloud Recognition via Folder Watcher')
    parser.add_argument('--cfg_file', type=str, required=True, help='Path to the model config yaml')
    parser.add_argument('--ckpt', type=str, required=True, help='Path to the model checkpoint')
    parser.add_argument('--watch_dir', type=str, default='./incoming_data', help='Directory to watch for new point clouds')
    parser.add_argument('--ext', type=str, default='.npy', help='File extension to watch (.npy or .bin)')
    
    args = parser.parse_args()
    
    # 初始化在线检测器 (模型驻留显存)
    detector = OnlineDetector(cfg_file=args.cfg_file, ckpt_file=args.ckpt)
    
    # 启动文件夹监控
    watch_and_infer(detector, args.watch_dir, args.ext)
