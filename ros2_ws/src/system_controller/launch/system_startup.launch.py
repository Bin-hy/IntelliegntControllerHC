import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package Shares
    duco_pkg = FindPackageShare('duco_ros_driver')
    vision_pkg = FindPackageShare('vision_server')
    system_pkg = FindPackageShare('system_controller')
    
    # Arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', 
        default_value='192.168.1.10',
        description='IP address of the robot controller'
    )
    
    # 1. Robot Driver
    # Assuming start_robot.launch.py takes arguments matching what we pass or we pass them specifically.
    # We pass 'robot_ip' if the included launch file supports it. 
    # If not, we might need to check the launch file content, but usually they do.
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([duco_pkg, '/launch/start_robot.launch.py']),
        launch_arguments={'robot_ip': LaunchConfiguration('robot_ip')}.items()
    )
    
    # 2. Vision System (Camera + Image Saver)
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vision_pkg, '/launch/vision_system.launch.py'])
    )
    
    # 3. Glove Node
    glove_node = Node(
        package='glove_node',
        executable='glove_node',
        name='glove_node',
        output='screen'
    )
    
    # 4. System Controller (The Brain)
    system_controller_node = Node(
        package='system_controller',
        executable='system_controller_node',
        name='system_controller',
        output='screen'
    )
    
    # 5. UI App (The Interface)
    # We start this last, maybe with a slight delay to ensure other services are ready
    ui_node = Node(
        package='ui_app',
        executable='ui_app',
        name='ui_app',
        output='screen'
    )
    
    ui_launch = TimerAction(
        period=3.0,
        actions=[ui_node]
    )

    return LaunchDescription([
        robot_ip_arg,
        robot_launch,
        vision_launch,
        glove_node,
        system_controller_node,
        ui_launch
    ])
