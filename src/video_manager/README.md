# Video Manager

The Video Manager is a ROS2 node that displays video frames with overlaid detection metadata. It subscribes to video frames and detections, caches frames for overlaying past detections, and provides real-time visualization.

## Overview

This module handles video display with detection overlays, managing frame caching for temporal visualization. It supports scaling for performance and smooth transitions between detection and normal display modes.

## Key Features

- Real-time video display with detection overlays
- Frame caching for past detection visualization
- Automatic scaling for CPU optimization
- Configurable display transitions
- ROS2 topic subscription for frames and detections

## Video Manager Notes

### 1. Frame Caching
`num_frames_cached` is attached to the frames FPS and detection delay. The cached frames are used to display detections in past overlaid.

### 2. Display Fallback
Once no detections are available and detection time out passed, the most recent frame is displayed instead.

### 3. Display Transition Control
The transition between detections display and normal display is controlled using `detection_time_out` variable.

### 4. Scaling Optimization
Scaling down only applies here for better CPU usage.

## Installation

1. Ensure ROS2 Humble and OpenCV are installed.
2. Build with `colcon build --packages-select video_manager`.

## Usage

Run the node:
```bash
ros2 run video_manager video_manager_node
```

Subscribed topics:
- `/stream_manager/current_frame` (shm_msgs::msg::Image1m)
- `/detections` (common_msgs::msg::Detections)

## Configuration

Configure caching, scaling, and timeout parameters in the node settings.

## Dependencies

- ROS2 Humble
- OpenCV
- shm_msgs
- common_msgs

## License

MIT License
