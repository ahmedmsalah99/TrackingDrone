# Detection and MOT

The Detection and MOT module is a ROS2 node that performs object detection using YOLO models and multi-object tracking using OC-Sort algorithm. It processes video frames from StreamManager and publishes detections with tracking information.

## Overview

This module integrates YOLO for object detection and OC-Sort for tracking, providing real-time detection and tracking capabilities in ROS2 environments. It handles frame processing, model inference, and publishes detection results with consistent timestamps.

## Key Features

- YOLO-based object detection (any version)
- OC-Sort multi-object tracking
- Configurable model parameters
- ROS2 topic publishing for detections
- Timestamp synchronization with frame delays

## Detection and MOT Notes

### 1. Tracking FPS Dependency
Tracking FPS is dependent on frame FPS. Ensure the input frame rate aligns with the tracking requirements for optimal performance.

### 2. OC-Sort Parameters
OC-Sort parameters depend on how fast your model is and what's your frames FPS is. Tune the tracking parameters based on model inference speed and input frame rate.

### 3. Delay Time
`delay_time` is the time we are assuming the model takes to get a prediction (always use maximum). It's a constant to keep everything smooth. Mainly used in discarding outdated detections and providing time stamps for our detections in a consistent way.

## Installation

1. Ensure ROS2 Humble and ONNX Runtime are installed.
2. Place the onnx model you want to use along with its labels inside the resources folder.
3. Place an "onnxruntime" folder in the root with the version you want to use.
4. Build with `colcon build --packages-select detection_and_mot`.

## Usage

Run the node:
```bash
ros2 run detection_and_mot detection_and_mot_node
```

Subscribed topics:
- `/stream_manager/current_frame` (shm_msgs::msg::Image1m)

Published topics:
- `/detections` (common_msgs::msg::Detections)

## Configuration

Configure model paths, tracking parameters, and delay settings in the node parameters.

## Dependencies

- ROS2 Humble
- ONNX Runtime
- OpenCV
- common_msgs

## License

MIT License
