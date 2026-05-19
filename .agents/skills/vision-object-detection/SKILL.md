---
name: "vision-object-detection"
description: "Computer vision algorithms (YOLO, ViT) for object detection and recognition. Invoke when processing camera feeds, detecting objects, or implementing visual perception for the excavator."
---

# Vision & Object Detection

This skill provides guidelines for integrating computer vision algorithms into the ROS2 excavator system.

## When to Invoke
- When processing image feeds from USB or network cameras (e.g., Hikvision).
- When implementing object detection using YOLO models (identifying trucks, dirt piles, obstacles).
- When applying Vision Transformers (ViT) for image classification or feature extraction.
- When publishing bounding boxes or segmented masks to ROS2 topics.

## Best Practices
1. **Model Deployment**: Use TensorRT, ONNX, or OpenVINO for accelerating YOLO/ViT models on edge devices (like Jetson or local GPUs) to ensure high FPS.
2. **ROS2 Integration**: Subscribe to `sensor_msgs/Image`, process with OpenCV/`cv_bridge`, and publish results using standard `vision_msgs/Detection2DArray` formats.
3. **Synchronization**: Synchronize vision data with IMU/joint states using `message_filters` for accurate spatial mapping and closed-loop control.