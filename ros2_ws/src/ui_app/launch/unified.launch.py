from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Args
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', 
        default_value='192.168.1.10',
        description='Robot IP Address'
    )

    # 1. Vision System (Camera + Image Saver)
    # Launches Orbbec Gemini 305 + Vision Server Node
    vision_pkg = FindPackageShare('vision_server')
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vision_pkg, '/launch/vision_system.launch.py'])
    )

    # 2. Robot Driver
    duco_pkg = FindPackageShare('duco_ros_driver')
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([duco_pkg, '/launch/start_robot.launch.py']),
        launch_arguments={'robot_ip': LaunchConfiguration('robot_ip')}.items() 
    )

    # 3. System Controller (Flow Control)
    sys_ctrl_node = Node(
        package='system_controller',
        executable='system_controller_node',
        name='system_controller',
        output='screen'
    )

    # 4. UI App
    # Launched last with a delay
    ui_node = Node(
        package='ui_app',
        executable='ui_app',
        name='ui_app',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip')
        }]
    )

    ui_delayed = TimerAction(
        period=5.0,
        actions=[ui_node]
    )

    return LaunchDescription([
        robot_ip_arg,
        vision_launch,
        robot_launch,
        sys_ctrl_node,
        ui_delayed
    ])
