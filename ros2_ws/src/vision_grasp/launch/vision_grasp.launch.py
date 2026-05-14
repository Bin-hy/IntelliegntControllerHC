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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def _hand_eye_yaml():
    return os.path.join(
        get_package_share_directory('vision_grasp'), 'config', 'hand_eye.yaml')


def _create_grasp_nodes(context):
    camera_ns_str = LaunchConfiguration('camera_ns').perform(context)
    sn = camera_ns_str.strip('/')

    import yaml
    cfg = {}
    yaml_path = _hand_eye_yaml()
    if os.path.exists(yaml_path):
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f)
    he = cfg.get('hand_eye', {})
    grasp = cfg.get('grasp', {})
    hand = cfg.get('hand', {})
    place = cfg.get('place', {})
    det = cfg.get('detection', {})
    ft = cfg.get('fingertip', {})

    # Static TF: L_base_link -> {sn}_link
    # Only when NOT calibrating (calibration node publishes its own)
    is_cal = LaunchConfiguration('calibration')
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_eye_tf',
        arguments=[
            '--x',  str(he.get('x',  0.05)),
            '--y',  str(he.get('y',  0.00)),
            '--z',  str(he.get('z',  0.08)),
            '--qx', str(he.get('qx', 0.0)),
            '--qy', str(he.get('qy', 0.0)),
            '--qz', str(he.get('qz', 0.0)),
            '--qw', str(he.get('qw', 1.0)),
            '--frame-id', 'L_base_link',
            '--child-frame-id', f'{sn}_link',
        ],
        condition=UnlessCondition(is_cal),
    )

    camera_optical_frame = f'{sn}_color_optical_frame'

    detector = Node(
        package='vision_grasp',
        executable='bottle_detector_node',
        name='bottle_detector',
        parameters=[{
            'camera_ns': camera_ns_str,
            'model_path': det.get('model_path', 'yolov8n.pt'),
            'confidence_threshold': det.get('confidence_threshold', 0.5),
            'depth_roi_half': det.get('depth_roi_half', 10),
            'min_depth_mm': det.get('min_depth_mm', 100.0),
            'max_depth_mm': det.get('max_depth_mm', 2000.0),
        }],
        output='screen',
    )

    coordinator = Node(
        package='vision_grasp',
        executable='grasp_coordinator_node',
        name='grasp_coordinator',
        parameters=[{
            'camera_ns': camera_ns_str,
            'camera_frame': camera_optical_frame,
            'base_frame': 'base_link',
            'pre_grasp_height': grasp.get('pre_grasp_height', 0.15),
            'grasp_z_offset':   grasp.get('grasp_z_offset',   0.02),
            'lift_height':      grasp.get('lift_height',       0.15),
            'grasp_rx':         grasp.get('grasp_rx',          3.14159),
            'grasp_ry':         grasp.get('grasp_ry',          0.0),
            'grasp_rz':         grasp.get('grasp_rz',          0.0),
            'move_speed':       grasp.get('move_speed',        0.20),
            'approach_speed':   grasp.get('approach_speed',    0.05),
            'lift_speed':       grasp.get('lift_speed',        0.10),
            'move_accel':       grasp.get('move_accel',        0.5),
            'hand_open':        hand.get('open',  [0]*6),
            'hand_close':       hand.get('close', [800]*6),
            'place_x':          place.get('x', 0.30),
            'place_y':          place.get('y', -0.30),
            'place_z':          place.get('z', 0.25),
            'grasp_offset_x':   cfg.get('grasp_offset', {}).get('x', 0.0),
            'grasp_offset_y':   cfg.get('grasp_offset', {}).get('y', 0.0),
            'grasp_offset_z':   cfg.get('grasp_offset', {}).get('z', 0.0),
            'fingertip_x':      ft.get('x', 0.0),
            'fingertip_y':      ft.get('y', 0.0),
            'fingertip_z':      ft.get('z', 0.12),
        }],
        output='screen',
    )

    calibration_node = Node(
        package='vision_grasp',
        executable='hand_eye_calibration_node',
        name='hand_eye_calibration',
        parameters=[{
            'camera_ns': camera_ns_str,
            'camera_frame': camera_optical_frame,
            'base_frame': 'base_link',
            'flange_frame': 'L_base_link',
        }],
        condition=IfCondition(is_cal),
        output='screen',
    )

    return [static_tf, detector, coordinator, calibration_node]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera_ns', default_value='/cam_305',
            description='Camera topic namespace'),
        DeclareLaunchArgument('launch_camera', default_value='false',
            description='Set true to launch camera too'),
        DeclareLaunchArgument('camera_name', default_value='cam_305',
            description='Camera name (only when launch_camera:=true)'),
        DeclareLaunchArgument('serial_number', default_value='',
            description='Camera serial (only when launch_camera:=true)'),
        DeclareLaunchArgument('calibration', default_value='false',
            description='Set true to launch calibration node'),
    ]

    camera_name = LaunchConfiguration('camera_name')
    serial_number = LaunchConfiguration('serial_number')
    launch_camera = LaunchConfiguration('launch_camera')

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

    grasp_nodes = OpaqueFunction(function=_create_grasp_nodes)

    return LaunchDescription(args + [camera_node, grasp_nodes])
