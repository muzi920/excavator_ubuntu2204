import argparse
import io
import time
from pathlib import Path
import numpy as np
import uvicorn
from fastapi import FastAPI, UploadFile, File, HTTPException
from fastapi.responses import JSONResponse

from online_detector import OnlineDetector

app = FastAPI(title="Point Cloud Online Recognition API")

# 全局变量存储检测器实例
detector = None

@app.post("/predict")
async def predict(file: UploadFile = File(...)):
    """
    接收上传的 .npy 文件，进行在线推理，并返回检测结果 (JSON格式)
    """
    if not file.filename.endswith('.npy'):
        raise HTTPException(status_code=400, detail="Only .npy files are supported.")
        
    try:
        contents = await file.read()
        # 从内存中读取 numpy 数组
        points = np.load(io.BytesIO(contents))
        
        # 维度检查和补齐 (确保为 Nx4)
        if len(points.shape) != 2:
            raise ValueError("Point cloud array must be 2D")
            
        if points.shape[1] < 4:
            padding = np.zeros((points.shape[0], 4 - points.shape[1]), dtype=points.dtype)
            points = np.hstack([points, padding])
        elif points.shape[1] > 4:
            points = points[:, :4]
            
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to parse npy file: {str(e)}")
        
    start_time = time.time()
    # 执行在线推理
    boxes, scores, labels = detector.inference(points)
    cost_time = time.time() - start_time
    
    # 构造返回结果
    response_data = {
        "inference_time_ms": round(cost_time * 1000, 2),
        "num_detections": len(boxes),
        "boxes": boxes.tolist(),     # 转换为普通 list 以便 JSON 序列化
        "scores": scores.tolist(),
        "labels": labels.tolist()
    }
    
    return JSONResponse(content=response_data)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Online Point Cloud Recognition API Server')
    parser.add_argument('--cfg_file', type=str, required=True, help='Path to the model config yaml')
    parser.add_argument('--ckpt', type=str, required=True, help='Path to the model checkpoint')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Host to listen on')
    parser.add_argument('--port', type=int, default=8000, help='Port to listen on')
    
    args = parser.parse_args()
    
    # 在服务启动前初始化模型
    print("Initializing Online Detector...")
    detector = OnlineDetector(cfg_file=args.cfg_file, ckpt_file=args.ckpt)
    print("Model initialized. Starting API server...")
    
    # 启动 FastAPI 服务
    uvicorn.run(app, host=args.host, port=args.port)
