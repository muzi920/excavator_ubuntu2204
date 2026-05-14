import cv2
import sys
import os

def main():
    # 海康威视 RTSP 流地址
    rtsp_url = "rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101"
    print(f"尝试连接海康摄像头 (RTSP over TCP): {rtsp_url}")
    
    # 海康摄像头通常建议使用 TCP 传输以保证画面稳定性，避免 udp 丢包花屏
    # 增加 stimeout 参数，设置超时时间为 3000000 微秒 (3秒)，防止卡死
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|stimeout;3000000"
    
    cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
    
    if not cap.isOpened():
        print(f"错误: 无法连接到海康 RTSP 流。请检查 IP、账号密码及网络连接。")
        sys.exit(1)
        
    print("成功连接到海康摄像头。按 'q' 键退出...")
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("错误: 读取流失败或流已结束。")
            break
            
        output_file = "hikvision_cam_snapshot.jpg"
        cv2.imwrite(output_file, frame)
        print(f"成功抓取一帧并保存为 {output_file}，程序自动退出。")
        break
            
    cap.release()

if __name__ == "__main__":
    main()
