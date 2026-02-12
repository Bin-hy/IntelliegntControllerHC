from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, Command

def generate_launch_description():
    # Args
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', 
        default_value='192.168.1.10',
        description='Robot IP Address'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='gcr14_1400',
        description='Duco robot model name'
    )

    ethercat_channel_arg = DeclareLaunchArgument(
        'ethercat_channel',
        default_value='1',
        description='EtherCAT network interface channel index'
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

    # Robot State Publisher (TF Bridge)
    # We launch it here because we cannot modify duco_ros_driver
    # We need to construct the URDF path correctly
    urdf_file_name = PythonExpression(["'duco_' + '", LaunchConfiguration('model'), "' + '.urdf'"])
    urdf_path = PathJoinSubstitution([
        FindPackageShare('duco_support'),
        'urdf',
        urdf_file_name
    ])
    
    # We need the CONTENT of the URDF for robot_state_publisher
    # Command will execute cat at runtime
    robot_description_content = ParameterValue(Command(['cat ', urdf_path]), value_type=str)
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 3. LHandPro Service
    lhand_pkg = FindPackageShare('lhandpro_service')
    lhand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lhand_pkg, '/launch/lhandpro.launch.py']),
        launch_arguments={'ethercat_channel': LaunchConfiguration('ethercat_channel')}.items()
    )

    # 4. System Controller (Flow Control)
    sys_ctrl_node = Node(
        package='system_controller',
        executable='system_controller_node',
        name='system_controller',
        output='screen'
    )

    # 5. UI App
    # Launched last with a delay
    
    # Construct URDF path for UI
    urdf_file_name = PythonExpression(["'duco_' + '", LaunchConfiguration('model'), "' + '.urdf'"])
    urdf_path = PathJoinSubstitution([
        FindPackageShare('duco_support'),
        'urdf',
        urdf_file_name
    ])

    ui_node = Node(
        package='ui_app',
        executable='ui_app',
        name='ui_app',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_urdf_path': urdf_path
        }]
    )

    ui_delayed = TimerAction(
        period=5.0,
        actions=[ui_node]
    )

    return LaunchDescription([
        robot_ip_arg,
        model_arg,
        ethercat_channel_arg,
        vision_launch,
        robot_launch,
        robot_state_publisher_node,
        lhand_launch,
        sys_ctrl_node,
        ui_delayed
    ])
