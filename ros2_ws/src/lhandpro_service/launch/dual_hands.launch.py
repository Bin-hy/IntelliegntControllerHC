from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define URDF paths
    left_urdf = '/home/user/IntelliegntControllerHC/ros2_ws/src/DH116_L_URDF/DH116-L000-A1/urdf/DH116-L000-A1.urdf'
    right_urdf = '/home/user/IntelliegntControllerHC/ros2_ws/src/DH116_R_URDF/DH116-R000-A1/urdf/DH116-R000-A1.urdf'

    # Define Joint Names
    # Assuming 11 active joints per hand as identified
    left_joint_names = [
        'L_finger11', 'L_finger12', 'L_finger13',
        'L_finger21', 'L_finger22',
        'L_finger31', 'L_finger32',
        'L_finger41', 'L_finger42',
        'L_finger51', 'L_finger52'
    ]

    right_joint_names = [
        'R_finger11', 'R_finger12', 'R_finger13',
        'R_finger21', 'R_finger22',
        'R_finger31', 'R_finger32',
        'R_finger41', 'R_finger42',
        'R_finger51', 'R_finger52'
    ]

    return LaunchDescription([
        # Left Hand Service
        Node(
            package='lhandpro_service',
            executable='lhandpro_service',
            name='lhandpro_service',
            namespace='left_hand',
            output='screen',
            parameters=[{
                'ethercat_channel': 0,
                'joint_names': left_joint_names
            }]
        ),
        # Right Hand Service
        Node(
            package='lhandpro_service',
            executable='lhandpro_service',
            name='lhandpro_service',
            namespace='right_hand',
            output='screen',
            parameters=[{
                'ethercat_channel': 1,
                'joint_names': right_joint_names
            }]
        ),
        
        # Robot State Publisher for Left Hand
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='left_hand_state_publisher',
            namespace='left_hand',
            output='screen',
            arguments=[left_urdf],
            remappings=[('joint_states', '/left_hand/joint_states')]
        ),
        
        # Robot State Publisher for Right Hand
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='right_hand_state_publisher',
            namespace='right_hand',
            output='screen',
            arguments=[right_urdf],
            remappings=[('joint_states', '/right_hand/joint_states')]
        ),

        # Static Transforms for Visualization (Bound to Robot Arm)
        # Bind Left Hand to link_6
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='left_hand_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'link_6', 'L_base_link']
        ),
        # Bind Right Hand to link_6
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='right_hand_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'link_6', 'R_base_link']
        )
    ])
