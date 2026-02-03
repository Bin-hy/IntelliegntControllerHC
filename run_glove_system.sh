#!/bin/bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# 启动 Glove Node (后台)
echo "Starting Glove Node..."
ros2 run glove_node glove_node &
GLOVE_PID=$!

sleep 2

# 启动 UI
echo "Starting UI..."
ros2 run ui_app ui_app

# UI 关闭后清理
kill $GLOVE_PID
