#!/bin/bash
# 启动标定节点的脚本
cd /home/hwtws/IntelliegntControllerHC/ros2_ws
source install/setup.bash
ros2 run vision_grasp hand_eye_calibration_node \
  --ros-args \
  -p camera_ns:=/CV2R1610004H \
  -p camera_frame:=CV2R1610004H_color_optical_frame \
  -p base_frame:=base_link \
  -p flange_frame:=L_base_link \
  -p marker_size:=0.1 \
  -p aruco_dict_id:=0 \
  -p aruco_marker_id:=0
