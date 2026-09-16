import cv2
import sys

def main():
    # 设备路径
    device_path = "/dev/video0"
    print(f"尝试打开 USB 摄像头: {device_path}")
    
    # 使用 OpenCV 的 VideoCapture 打开设备
    # 在 Linux 下通常使用 V4L2 (Video for Linux 2) 后端
    cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print(f"错误: 无法打开摄像头 {device_path}，请检查设备是否连接或是否有权限访问该设备。")
        sys.exit(1)
        
    print("成功打开摄像头。按 'q' 键退出...")
    
    while True:
        # 读取一帧图像
        ret, frame = cap.read()
        
        if not ret:
            print("错误: 无法读取图像帧，可能连接已断开。")
            break
            
        # 保存图像代替 cv2.imshow
        output_file = "usb_cam_snapshot.jpg"
        cv2.imwrite(output_file, frame)
        print(f"成功抓取一帧并保存为 {output_file}，程序自动退出。")
        break
            
    # 释放资源
    cap.release()

if __name__ == "__main__":
    main()
