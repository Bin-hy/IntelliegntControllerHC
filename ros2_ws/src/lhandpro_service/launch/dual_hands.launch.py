import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    lhand_pkg = FindPackageShare('lhandpro_description')
    
    # Use PathJoinSubstitution to dynamically find the URDF files
    left_urdf = PathJoinSubstitution([lhand_pkg, 'urdf', 'DH126S-L000-A1.urdf'])
    right_urdf = PathJoinSubstitution([lhand_pkg, 'urdf', 'DH126S-R000-A1.urdf'])

    # Joint names for left and right hands
    joint_names_l = [
        "L_H1_joint", "L_H2_joint", "L_H3_joint", "L_H4_joint", "L_H5_joint", "L_H6_joint"
    ]
    
    joint_names_r = [
        "R_H1_joint", "R_H2_joint", "R_H3_joint", "R_H4_joint", "R_H5_joint", "R_H6_joint"
    ]

    # Node for Left Hand
    left_hand_node = Node(
        package='lhandpro_service',
        executable='hand_control_service',
        name='left_hand_node',
        output='screen',
        parameters=[
            {'port_name': '/dev/ttyUSB0'},
            {'baud_rate': 115200},
            {'node_name': 'left_hand_node'},
            {'joint_names': joint_names_l},
            {'urdf_path': left_urdf}
        ]
    )

    # Node for Right Hand
    right_hand_node = Node(
        package='lhandpro_service',
        executable='hand_control_service',
        name='right_hand_node',
        output='screen',
        parameters=[
            {'port_name': '/dev/ttyUSB1'},
            {'baud_rate': 115200},
            {'node_name': 'right_hand_node'},
            {'joint_names': joint_names_r},
            {'urdf_path': right_urdf}
        ]
    )

    return LaunchDescription([
        LogInfo(msg="Launching Dual Hand Control Services..."),
        left_hand_node,
        right_hand_node
    ])
