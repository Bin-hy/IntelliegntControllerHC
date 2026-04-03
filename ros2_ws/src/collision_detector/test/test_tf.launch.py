"""
test_tf.launch.py — 发布假的机械臂 TF 用于测试碰撞检测

无需真实机械臂，使用 robot_state_publisher + static joint_states
模拟一个固定姿态的机械臂 TF 树。

使用方式:
  ros2 launch collision_detector test_tf.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def generate_launch_description():
    # 使用已有的机械臂 URDF
    urdf_file = os.path.join(
        FindPackageShare('duco_support').find('duco_support'),
        'urdf', 'duco_gcr5_910.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # robot_state_publisher: 从 URDF 生成 TF
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='test_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }],
    )

    # joint_state_publisher: 发布固定关节角度
    # 这里发布一个"手臂伸直向前"的姿态，方便测试
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='test_joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': [],
            # 默认零位：机械臂直立
        }],
    )

    return LaunchDescription([
        robot_state_pub,
        joint_state_pub,
    ])
