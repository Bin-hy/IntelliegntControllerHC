"""Launch file for glove_hand_bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hand_namespace',
            default_value='lhandpro_service',
            description='Namespace of the lhandpro service node'),
        DeclareLaunchArgument(
            'hand_side',
            default_value='left',
            description='Which glove hand to use: left or right'),
        DeclareLaunchArgument(
            'control_rate_hz',
            default_value='20.0',
            description='Control loop frequency in Hz'),
        DeclareLaunchArgument(
            'smoothing_alpha',
            default_value='0.3',
            description='EMA smoothing factor (0=no update, 1=no filter)'),
        DeclareLaunchArgument(
            'deadzone_deg',
            default_value='2.0',
            description='Ignore angle changes below this threshold (degrees)'),
        DeclareLaunchArgument(
            'enabled_at_start',
            default_value='false',
            description='Enable control at startup (true/false)'),
        DeclareLaunchArgument(
            'glove_flex_min',
            default_value='0.0',
            description='Glove flexion min angle (degrees, fully open)'),
        DeclareLaunchArgument(
            'glove_flex_max',
            default_value='90.0',
            description='Glove flexion max angle (degrees, fully closed)'),
        DeclareLaunchArgument(
            'glove_spread_min',
            default_value='-30.0',
            description='Glove spread min angle (degrees)'),
        DeclareLaunchArgument(
            'glove_spread_max',
            default_value='30.0',
            description='Glove spread max angle (degrees)'),

        Node(
            package='glove_hand_bridge',
            executable='glove_hand_bridge',
            name='glove_hand_bridge',
            output='screen',
            parameters=[{
                'hand_namespace': LaunchConfiguration('hand_namespace'),
                'hand_side': LaunchConfiguration('hand_side'),
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
                'deadzone_deg': LaunchConfiguration('deadzone_deg'),
                'enabled_at_start': LaunchConfiguration('enabled_at_start'),
                'glove_flex_min': LaunchConfiguration('glove_flex_min'),
                'glove_flex_max': LaunchConfiguration('glove_flex_max'),
                'glove_spread_min': LaunchConfiguration('glove_spread_min'),
                'glove_spread_max': LaunchConfiguration('glove_spread_max'),
            }],
        ),
    ])
