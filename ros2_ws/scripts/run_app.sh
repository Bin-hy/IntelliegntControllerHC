#!/bin/bash
# Wrapper script to launch the Intelligent Controller System
# This script sets up the ROS2 environment and launches the main application

# 1. Source System ROS2 Environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Error: ROS2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    read -p "Press enter to exit..."
    exit 1
fi

# 2. Source Workspace Environment
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SETUP_FILE="$WORKSPACE_DIR/install/setup.bash"

if [ -f "$SETUP_FILE" ]; then
    source "$SETUP_FILE"
else
    echo "Error: Workspace setup file not found at $SETUP_FILE"
    echo "Please build the workspace first: colcon build"
    read -p "Press enter to exit..."
    exit 1
fi

# 3. Launch the Application
echo "Starting Intelligent Controller System..."
echo "Workspace: $WORKSPACE_DIR"

# Using unified.launch.py as the main entry point
ros2 launch ui_app unified.launch.py

# 4. Cleanup/Exit
echo "Application exited."
read -p "Press enter to close..."
