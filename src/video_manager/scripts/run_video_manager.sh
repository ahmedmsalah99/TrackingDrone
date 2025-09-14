#!/bin/bash

# VideoManager run script
# This script demonstrates how to run the VideoManager node

echo "Starting VideoManager..."
echo "Make sure StreamManager is running and publishing to:"
echo "  - /stream_manager/current_frame"
echo "  - /stream_manager/delayed_frame"
echo ""

# Source the workspace
source ../../../install/setup.bash

# Run the VideoManager node
ros2 run VideoManager video_manager_node