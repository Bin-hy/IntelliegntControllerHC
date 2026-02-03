from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.192.10',
        description='IP address of the DUCO robot controller'
    )
    
    arm_num_arg = DeclareLaunchArgument(
        'arm_num',
        default_value='1',
        description='Number of robot arms'
    )
    
    arm_dof_arg = DeclareLaunchArgument(
        'arm_dof',
        default_value='6',
        description='Degrees of freedom of the robot'
    )

    # Node 1: Robot Status (Publishes robot state to /duco_cobot/robot_state)
    duco_robot_status_node = Node(
        package='duco_ros_driver',
        executable='DucoRobotStatus',
        name='duco_robot_status',
        output='screen',
        parameters=[{
            'arm_num': LaunchConfiguration('arm_num'),
            'arm_dof': LaunchConfiguration('arm_dof'),
            'server_host_1': LaunchConfiguration('robot_ip')
        }]
    )

    # Node 2: Robot Control (Provides control services: RobotControl, RobotMove, RobotIoControl)
    duco_robot_control_node = Node(
        package='duco_ros_driver',
        executable='DucoRobotControl',
        name='duco_robot_control',
        output='screen',
        parameters=[{
            'arm_num': LaunchConfiguration('arm_num'),
            'arm_dof': LaunchConfiguration('arm_dof'),
            'server_host_1': LaunchConfiguration('robot_ip')
        }]
    )

    # Node 3: Duco Driver (Handles MoveIt trajectory execution)
    duco_driver_node = Node(
        package='duco_ros_driver',
        executable='DucoDriver',
        name='duco_driver',
        output='screen',
        parameters=[{
            'arm_num': LaunchConfiguration('arm_num'),
            'server_host_1': LaunchConfiguration('robot_ip')
        }]
    )

    return LaunchDescription([
        robot_ip_arg,
        arm_num_arg,
        arm_dof_arg,
        duco_robot_status_node,
        duco_robot_control_node,
        duco_driver_node
    ])
