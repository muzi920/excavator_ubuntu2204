import cv2
import sys
import os

def main():
    # RTSP 流地址列表
    rtsp_urls = [
        "rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream",
        "rtsp://admin:@192.168.158.103:554/stream"
    ]
    
    # 针对 RTSP 流，为了降低延迟和防止花屏，设置 FFmpeg 环境变量
    # 增加 stimeout 参数，设置超时时间为 3000000 微秒 (3秒)，防止卡死
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|stimeout;3000000"
    
    for idx, rtsp_url in enumerate(rtsp_urls):
        print(f"尝试连接网络摄像头{idx+1} (RTSP): {rtsp_url}")
        cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        
        if not cap.isOpened():
            print(f"错误: 无法连接到 RTSP 流 {rtsp_url}。请检查 IP、端口、账号密码及网络连接。")
            continue
            
        print(f"成功连接到摄像头{idx+1}流。")
        
        ret, frame = cap.read()
        
        if not ret:
            print(f"错误: 读取流 {rtsp_url} 失败或流已结束。")
            cap.release()
            continue
            
        output_file = f"network_cam{idx+1}_snapshot.jpg"
        cv2.imwrite(output_file, frame)
        print(f"成功抓取一帧并保存为 {output_file}。")
        
        cap.release()
        
    print("程序自动退出。")

if __name__ == "__main__":
    main()
