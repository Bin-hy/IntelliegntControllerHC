from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    ethercat_channel_arg = DeclareLaunchArgument(
        'ethercat_channel',
        default_value='1',
        description='EtherCAT network interface channel index'
    )

    return LaunchDescription([
        ethercat_channel_arg,
        Node(
            package='lhandpro_service',         # 包名
            executable='lhandpro_service',  # 可执行文件名
            name='lhandpro_service', # 节点名称
            namespace='/lhandpro_service',      # 设置命名空间
            output='screen',                 # 输出日志到终端
            emulate_tty=True,                 # 更好地显示日志
            parameters=[{
                'ethercat_channel': LaunchConfiguration('ethercat_channel')
            }]            
        )
    ])