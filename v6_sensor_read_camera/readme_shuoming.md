1.usb摄像头
驱动：
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video2"

2.网络摄像头：192.168.158.102
ffplay rtsp://admin:GWWzPzb2Tci@192.168.158.102:554/stream
3.海康：192.168.158.101
ffplay -rtsp_transport tcp "rtsp://admin:GWWzPzb2Tci@192.168.158.101:554/Streaming/Channels/101"

电脑ip：192.168.158.15