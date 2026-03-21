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
        default_value='gcr5_910',
        description='Duco robot model name'
    )

    protocol_arg = DeclareLaunchArgument(
        'protocol',
        default_value='tool_rs485',
        description='LHand communication protocol: tool_rs485, rs485, or can'
    )

    camera1_serial_arg = DeclareLaunchArgument(
        'camera1_serial',
        default_value='',
        description='Serial number of camera 1 (e.g. CV2R1610004Y). Empty = first detected device.'
    )

    camera2_serial_arg = DeclareLaunchArgument(
        'camera2_serial',
        default_value='',
        description='Serial number of camera 2 (e.g. CV2R1610004H). Empty = single camera mode.'
    )

    # 1. Vision System (Camera + Image Saver)
    # Launches Orbbec Gemini 330 + Vision Server Node
    vision_pkg = FindPackageShare('vision_server')
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vision_pkg, '/launch/vision_system.launch.py']),
        launch_arguments={
            'camera1_serial': LaunchConfiguration('camera1_serial'),
            'camera2_serial': LaunchConfiguration('camera2_serial'),
        }.items()
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
    urdf_file_name = PythonExpression([
        "'duco_gcr5_910_with_dh116_lhand.urdf' if '",
        LaunchConfiguration('model'),
        "' == 'gcr5_910' else 'duco_' + '",
        LaunchConfiguration('model'),
        "' + '.urdf'"
    ])
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
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'protocol': LaunchConfiguration('protocol'),
        }.items()
    )

    # LHand Description & State Publisher
    lhand_desc_pkg = FindPackageShare('dh116_l000_a1')
    lhand_urdf_path = PathJoinSubstitution([lhand_desc_pkg, 'urdf', 'DH116-L000-A1.urdf'])
    rhand_desc_pkg = FindPackageShare('dh116_r000_a1')
    rhand_urdf_path = PathJoinSubstitution([rhand_desc_pkg, 'urdf', 'DH116-R000-A1.urdf'])

    # Hand joint state publisher (publishes L_finger* names to /joint_states)
    # The combined robot_state_publisher above handles TF for the full chain
    # (base_link -> link_6 -> L_base_link -> L_finger*_Link)
    lhand_state_publisher = Node(
        package='lhandpro_description',
        executable='lhandpro_state_publisher',
        name='lhandpro_state_publisher',
        output='screen',
        parameters=[{'joint_prefix': 'L_'}]
    )

    # 4. System Controller (Flow Control)
    sys_ctrl_node = Node(
        package='system_controller',
        executable='system_controller_node',
        name='system_controller',
        output='screen'
    )

    # 4b. Collision Detector (Vision-based, runs alongside system_controller)
    collision_detector_node = Node(
        package='collision_detector',
        executable='collision_detector_node',
        name='collision_detector',
        output='screen',
        parameters=[{
            'point_cloud_topic': '/camera/depth/points',
            'link_names': ['base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'],
            'link_radii': [0.15, 0.12, 0.12, 0.10, 0.10, 0.10, 0.08],
            'self_filter_margin': 0.04,
            'emergency_threshold': 0.03,
            'warning_threshold': 0.10,
            'caution_threshold': 0.20,
            'detection_hz': 10.0,
            'downsample_stride': 6,
            'auto_pause': True,
        }]
    )

    # 5. UI App
    # Launched last with a delay
    
    # Construct URDF path for UI
    urdf_file_name = PythonExpression([
        "'duco_gcr5_910_with_dh116_lhand.urdf' if '",
        LaunchConfiguration('model'),
        "' == 'gcr5_910' else 'duco_' + '",
        LaunchConfiguration('model'),
        "' + '.urdf'"
    ])
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
            'robot_urdf_path': urdf_path,
            'left_hand_urdf_path': lhand_urdf_path,
            'right_hand_urdf_path': rhand_urdf_path,
            'robot_model': LaunchConfiguration('model')
        }]
    )

    ui_delayed = TimerAction(
        period=3.0,
        actions=[ui_node]
    )

    return LaunchDescription([
        robot_ip_arg,
        model_arg,
        protocol_arg,
        camera1_serial_arg,
        camera2_serial_arg,
        vision_launch,
        robot_launch,
        robot_state_publisher_node,
        lhand_launch,
        lhand_state_publisher,
        sys_ctrl_node,
        collision_detector_node,
        ui_delayed
    ])
