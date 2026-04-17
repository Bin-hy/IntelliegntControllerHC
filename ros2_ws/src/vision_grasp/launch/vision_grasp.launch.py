#!/usr/bin/env python3
"""
vision_grasp.launch.py
----------------------
Usage:
  # Camera already running (from unified.launch.py):
  ros2 launch vision_grasp vision_grasp.launch.py camera_ns:=/CV2R1610004H

  # Launch camera too (standalone):
  ros2 launch vision_grasp vision_grasp.launch.py launch_camera:=true camera_name:=cam_305
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _create_grasp_nodes(context):
    """Derive TF frame names from camera_ns at launch time and create nodes."""
    camera_ns_str = LaunchConfiguration('camera_ns').perform(context)
    # Strip leading slash to get the camera name used by OrbbecSDK for frame IDs
    # e.g. "/CV2R1610004H" → "CV2R1610004H"
    sn = camera_ns_str.strip('/')

    # --- Static TF: link_6 → {sn}_link ---
    # This BRIDGES the robot TF tree (base_link → ... → link_6) to the camera
    # TF tree ({sn}_link → {sn}_depth_frame → {sn}_color_optical_frame).
    # The Orbbec driver publishes the internal camera chain automatically.
    # Translation/rotation here represent the physical mounting of the camera
    # on the robot flange — adjust after proper hand-eye calibration.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_eye_tf',
        arguments=[
            '--x', '0.05', '--y', '0.0', '--z', '0.08',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'link_6', '--child-frame-id', f'{sn}_link',
        ],
    )

    # The camera_frame for the coordinator must be the optical frame where
    # pinhole-deprojected 3D coordinates live (Z-forward, X-right, Y-down).
    # When depth_registration=true, depth is aligned to color, so we use
    # the color optical frame.
    camera_optical_frame = f'{sn}_color_optical_frame'

    # --- Bottle detector ---
    detector = Node(
        package='vision_grasp',
        executable='bottle_detector_node',
        name='bottle_detector',
        parameters=[{
            'camera_ns': camera_ns_str,
            'model_path': 'yolov8n.pt',
            'confidence_threshold': 0.5,
            'depth_roi_half': 10,
            'min_depth_mm': 100.0,
            'max_depth_mm': 2000.0,
        }],
        output='screen',
    )

    # --- Grasp coordinator ---
    coordinator = Node(
        package='vision_grasp',
        executable='grasp_coordinator_node',
        name='grasp_coordinator',
        parameters=[{
            'camera_ns': camera_ns_str,
            'camera_frame': camera_optical_frame,
            'base_frame': 'base_link',
            'pre_grasp_height': 0.15,
            'grasp_z_offset': 0.02,
            'lift_height': 0.15,
            'grasp_rx': 3.14159,
            'grasp_ry': 0.0,
            'grasp_rz': 0.0,
            'move_speed': 0.20,
            'approach_speed': 0.05,
            'lift_speed': 0.10,
            'move_accel': 0.5,
            'hand_open': [0, 0, 0, 0, 0, 0],
            'hand_close': [800, 800, 800, 800, 800, 800],
            'place_x': 0.30,
            'place_y': -0.30,
            'place_z': 0.25,
        }],
        output='screen',
    )

    return [static_tf, detector, coordinator]


def generate_launch_description():
    # --- Arguments ---
    args = [
        DeclareLaunchArgument('camera_ns', default_value='/cam_305',
            description='Camera topic namespace (use SN if already launched by main system)'),
        DeclareLaunchArgument('launch_camera', default_value='false',
            description='Set true to launch Gemini 305 camera (skip if main system already launched it)'),
        DeclareLaunchArgument('camera_name', default_value='cam_305',
            description='Camera name (only used when launch_camera:=true)'),
        DeclareLaunchArgument('serial_number', default_value='',
            description='Camera serial number (only used when launch_camera:=true)'),
    ]

    camera_name = LaunchConfiguration('camera_name')
    serial_number = LaunchConfiguration('serial_number')
    launch_camera = LaunchConfiguration('launch_camera')

    # --- 1. Gemini 305 camera (conditional) ---
    orbbec_share = get_package_share_directory('orbbec_camera')
    gemini305_launch = os.path.join(orbbec_share, 'launch', 'gemini305.launch.py')
    if not os.path.exists(gemini305_launch):
        gemini305_launch = os.path.join(
            os.path.dirname(__file__), '..', '..', '..',
            'OrbbecSDK_ROS2', 'orbbec_camera', 'launch', 'gemini305.launch.py')
        gemini305_launch = os.path.abspath(gemini305_launch)

    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gemini305_launch),
        launch_arguments={
            'camera_name': camera_name,
            'serial_number': serial_number,
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_point_cloud': 'false',
            'depth_registration': 'true',
        }.items(),
        condition=IfCondition(launch_camera),
    )

    # --- 2-4. Static TF + Detector + Coordinator (created via OpaqueFunction) ---
    grasp_nodes = OpaqueFunction(function=_create_grasp_nodes)

    return LaunchDescription(args + [camera_node, grasp_nodes])
