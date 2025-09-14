# VideoManager

A ROS2 package for displaying video frames with metadata overlay from StreamManager topics.

## Overview

VideoManager subscribes to video frame topics published by StreamManager and displays them in OpenCV windows with overlaid metadata information. It provides real-time visualization of both current and delayed video frames.

## Features

- Subscribes to `/stream_manager/current_frame` and `/stream_manager/delayed_frame` topics
- Displays frames in separate OpenCV windows with metadata overlay
- 20ms timer-based display updates (50 Hz)
- Thread-safe frame handling
- Automatic frame resizing for display
- Real-time metadata including:
  - Frame type (CURRENT/DELAYED)
  - Frame ID and timestamp
  - Frame dimensions
  - Frame age (time since capture)
  - Current display time

## Dependencies

- ROS2 (tested with Humble)
- OpenCV
- shm_msgs (shared memory message package)

## Topics

### Subscribed Topics

- `/stream_manager/current_frame` (shm_msgs/msg/Image1m): Current video frames from StreamManager
- `/stream_manager/delayed_frame` (shm_msgs/msg/Image1m): Delayed video frames from StreamManager

## Usage

### Building

```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select VideoManager
```

### Running

```bash
# Source the workspace
source install/setup.bash

# Run the VideoManager node directly
ros2 run VideoManager video_manager_node

# Or use the launch file
ros2 launch VideoManager video_manager.launch.py
```

### Launch with StreamManager

Make sure StreamManager is running and publishing to the required topics:

```bash
# Terminal 1: Run StreamManager
ros2 run StreamManager stream_manager_node

# Terminal 2: Run VideoManager
ros2 run VideoManager video_manager_node
```

## Configuration

The VideoManager uses the following default parameters:

- Display update rate: 50 Hz (20ms timer)
- Maximum display size: 640x480 pixels
- QoS: KeepLast(10)

## Display Windows

The package creates two OpenCV windows:

1. **Current Frame**: Displays the latest frames from `/stream_manager/current_frame`
2. **Delayed Frame**: Displays the delayed frames from `/stream_manager/delayed_frame`

Each window shows the frame with overlaid metadata including timestamp, frame ID, dimensions, and age information.

## Implementation Details

- Uses shared memory messages (shm_msgs) for efficient image transport
- Thread-safe frame storage with mutex protection
- Automatic frame scaling for display purposes
- Real-time metadata overlay with green text on black background
- Proper resource cleanup on shutdown

## Troubleshooting

### No frames displayed
- Ensure StreamManager is running and publishing frames
- Check topic names match: `/stream_manager/current_frame` and `/stream_manager/delayed_frame`
- Verify shm_msgs package is properly installed

### Performance issues
- The display timer runs at 50 Hz (20ms) as specified
- Frame processing is optimized for real-time display
- Large frames are automatically resized for display

### OpenCV window issues
- Ensure X11 forwarding is enabled if running over SSH
- Check OpenCV installation and display environment