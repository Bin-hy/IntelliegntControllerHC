from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("dual_arms", package_name="duco_arms").to_moveit_configs()
    ld = LaunchDescription()
    ld.add_action(
    	DeclareLaunchArgument(
    	'arm_num', 
    	default_value='2',
    	)
    )
    ld.add_action(
    	DeclareLaunchArgument(
    	'server_host_1', 
    	default_value='192.168.120.138',
    	)
    )
    ld.add_action(
    	DeclareLaunchArgument(
    	'server_host_2', 
    	default_value='192.168.120.225',
    	)
    )
    ld.add_action(
	Node(
             name="robot_control",
             package="robot_control",
             executable="robot_control",
             output="screen",
             parameters=[
               moveit_config.robot_description,
               moveit_config.robot_description_semantic,
               moveit_config.robot_description_kinematics,
               {'arm_num':LaunchConfiguration('arm_num')},
               {'server_host_1':LaunchConfiguration('server_host_1')},
               {'server_host_2':LaunchConfiguration('server_host_2')}
             ]
            )
    )
    return ld
