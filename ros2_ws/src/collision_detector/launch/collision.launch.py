"""
collision_detector launch 文件

使用方式:
  # 独立启动 (假设相机和机械臂已在运行):
  ros2 launch collision_detector collision.launch.py

  # 指定点云话题 (例如带命名空间的相机):
  ros2 launch collision_detector collision.launch.py point_cloud_topic:=/CV2R1610004Y/depth/points

  # 同时启动 RViz:
  ros2 launch collision_detector collision.launch.py rviz:=true

设计说明:
  - 与 unified.launch.py 解耦: collision_detector 可以独立启动或被 include
  - 参数从 YAML 文件加载, launch arg 可覆盖关键参数
  - RViz 可选启动, 生产环境通常不需要
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('collision_detector')

    # ---- Launch Arguments ----
    args = [
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/camera/depth/points',
            description='点云输入话题'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='是否启动 RViz 可视化'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                pkg_share, 'config', 'collision_params.yaml'
            ]),
            description='参数配置文件路径'
        ),
    ]

    # ---- Collision Detector Node ----
    collision_node = Node(
        package='collision_detector',
        executable='collision_detector_node',
        name='collision_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            # launch arg 可覆盖 YAML 中的值
            {'point_cloud_topic': LaunchConfiguration('point_cloud_topic')},
        ],
        # 确保日志等级合理: INFO 平时, DEBUG 调试
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # ---- RViz (可选) ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='collision_rviz',
        arguments=['-d', PathJoinSubstitution([
            pkg_share, 'rviz', 'collision.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [collision_node, rviz_node])
