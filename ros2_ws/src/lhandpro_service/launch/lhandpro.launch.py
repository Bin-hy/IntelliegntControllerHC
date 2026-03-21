from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='DUCO controller IP address'
    )

    protocol_arg = DeclareLaunchArgument(
        'protocol',
        default_value='tool_rs485',
        description='Communication protocol: tool_rs485, rs485, or can'
    )

    return LaunchDescription([
        robot_ip_arg,
        protocol_arg,
        Node(
            package='lhandpro_service',         # 包名
            executable='lhandpro_service',  # 可执行文件名
            name='lhandpro_service', # 节点名称
            namespace='/lhandpro_service',      # 设置命名空间
            output='screen',                 # 输出日志到终端
            emulate_tty=True,                 # 更好地显示日志
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'protocol': LaunchConfiguration('protocol')
            }]
        )
    ])
