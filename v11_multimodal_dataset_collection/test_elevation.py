import numpy as np
import cv2
import os

def generate_elevation_map(points, x_range=(-2.0, 2.0), y_range=(-2.0, 2.0), resolution=0.02, z_range=(-0.4, 0.7)):
    """
    点云高程图算法 (Elevation Map) - 独立测试版
    """
    # 1. 过滤 X, Y 和 Z
    mask = (points[:, 0] >= x_range[0]) & (points[:, 0] < x_range[1]) & \
           (points[:, 1] >= y_range[0]) & (points[:, 1] < y_range[1]) & \
           (points[:, 2] >= z_range[0])
    pts = points[mask]

    width = int((x_range[1] - x_range[0]) / resolution)
    height = int((y_range[1] - y_range[0]) / resolution)
    
    # 2. 初始化一个值为 z_range[0] 的一维数组，代表无数据或最低点
    flat_map = np.full(width * height, z_range[0], dtype=np.float32)
    
    if len(pts) > 0:
        # 3. 映射到像素坐标 (u, v)
        # 注意：这里我们让 u 对应 Y 轴，v 对应 X 轴（或者反过来），根据一般图像的坐标系来定
        # 为了与网易伏羲数据集兼容：通常 X 是前方，Y 是左方
        # 图像中，往往希望前方朝上。所以 u(列) = Y, v(行) = -X
        
        # 简单映射：u 从左到右对应 Y 从小到大，v 从上到下对应 X 从大到小
        u = np.floor((pts[:, 1] - y_range[0]) / resolution).astype(int)
        v = np.floor((x_range[1] - pts[:, 0]) / resolution).astype(int) - 1 # X反向，确保前方在图像上方
        z = pts[:, 2]
        
        valid_idx = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u = u[valid_idx]
        v = v[valid_idx]
        z = z[valid_idx]
        
        # 4. 计算一维索引并取最大 Z 值
        flat_indices = v * width + u
        np.maximum.at(flat_map, flat_indices, z)
        
    elevation_map = flat_map.reshape((height, width))
    
    # 5. 归一化到 0-255 灰度值
    z_min, z_max = z_range
    elevation_map = np.clip(elevation_map, z_min, z_max)
    elevation_map_img = ((elevation_map - z_min) / (z_max - z_min) * 255.0).astype(np.uint8)
    
    # 6. 转为 3 通道纯灰度图 (对齐网易的 3 通道格式，R=G=B)
    elevation_map_3ch = cv2.cvtColor(elevation_map_img, cv2.COLOR_GRAY2BGR)
    
    return elevation_map_3ch

def main():
    print("=== 开始高程图 (Elevation Map) 测试 ===")
    
    # 模拟生成一个点云
    # 1. 基础地面 z = -0.1
    x_ground = np.random.uniform(-2, 2, 50000)
    y_ground = np.random.uniform(-2, 2, 50000)
    z_ground = np.full(50000, -0.1)
    
    # 2. 正前方有一个土包 (x: 1.0, y: 0.0)，高度为 1.0
    x_bump = np.random.normal(1.0, 0.2, 10000)
    y_bump = np.random.normal(0.0, 0.2, 10000)
    # 高度呈高斯分布，最高点在 1.0 左右
    z_bump = 1.0 - ( (x_bump - 1.0)**2 + y_bump**2 ) * 2.0
    z_bump = np.clip(z_bump, -0.1, 1.5)
    
    # 3. 左侧有一堵墙 (x: -1 到 1, y: 1.5)，高度为 1.2
    x_wall = np.random.uniform(-1, 1, 10000)
    y_wall = np.random.uniform(1.4, 1.6, 10000)
    z_wall = np.random.uniform(0.0, 1.2, 10000)
    
    # 合并点云
    pts = np.vstack((
        np.column_stack((x_ground, y_ground, z_ground)),
        np.column_stack((x_bump, y_bump, z_bump)),
        np.column_stack((x_wall, y_wall, z_wall))
    ))
    
    print(f"生成的模拟点云总数: {len(pts)}")
    
    # 转换为高程图
    elevation_img = generate_elevation_map(pts, resolution=0.02)
    
    print(f"生成的高程图尺寸: {elevation_img.shape}, 最大值: {elevation_img.max()}, 最小值: {elevation_img.min()}")
    
    # 保存图像到当前执行目录 (v11_multimodal_dataset_collection)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    save_path = os.path.join(current_dir, "test_elevation_map_result.png")
    cv2.imwrite(save_path, elevation_img)
    print(f"高程图已保存至: {save_path}")

if __name__ == "__main__":
    main()
