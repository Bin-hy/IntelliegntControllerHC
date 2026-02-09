#!/bin/bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# 启动所有节点
echo "Starting ui Node..."
ros2 launch ui_app unified.launch.py &
GLOVE_PID=$!

# UI 关闭后清理
kill $GLOVE_PID
