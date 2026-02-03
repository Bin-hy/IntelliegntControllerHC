from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sequence_demo_cpp',         # 包名
            executable='sequence_caller',  # 可执行文件名
            name='sequence_caller', # 节点名称
            namespace='/lhandpro_service',      # 设置命名空间
            output='screen',                 # 输出日志到终端
            emulate_tty=True                 # 更好地显示日志
        )
    ])