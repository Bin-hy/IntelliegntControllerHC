from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, Command
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

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

    ethercat_channel_arg = DeclareLaunchArgument(
        'ethercat_channel',
        default_value='1',
        description='EtherCAT network interface channel index'
    )

    hand_side_arg = DeclareLaunchArgument(
        'hand_side',
        default_value='left',
        description='Which DH116 hand is connected: left or right'
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

    depth_camera_serial_arg = DeclareLaunchArgument(
        'depth_camera_serial',
        default_value='',
        description='Serial number of depth measurement camera (e.g. AYZ8953002L / Gemini 215). Empty = auto-detect 3rd camera.'
    )

    lift_serial_port_arg = DeclareLaunchArgument(
        'lift_serial_port',
        default_value='/dev/ttyUSB0',
        description='RS485 serial port for lift platform servo driver'
    )

    grasp_camera_ns_arg = DeclareLaunchArgument(
        'grasp_camera_ns',
        default_value='/CV2R1610004H',
        description='Camera namespace for vision grasp (e.g. /CV2R1610004H). Empty = disable vision grasp.'
    )

    enable_calibration_arg = DeclareLaunchArgument(
        'enable_calibration',
        default_value='false',
        description='Set true to launch hand-eye calibration node (requires printed ArUco marker)'
    )

    enable_grasp = PythonExpression([
        "'", LaunchConfiguration('grasp_camera_ns'), "' != ''"
    ])

    # ---- Derived substitutions based on hand_side ----
    hand_side = LaunchConfiguration('hand_side')

    # Namespace: "left" -> "/lhandpro_service", "right" -> "/rhandpro_service"
    hand_namespace = PythonExpression([
        "'lhandpro_service' if '", hand_side, "' == 'left' else 'rhandpro_service'"
    ])

    # Joint prefix: "left" -> "L_", "right" -> "R_"
    hand_joint_prefix = PythonExpression([
        "'L_' if '", hand_side, "' == 'left' else 'R_'"
    ])

    # 1. Vision System (Camera + Image Saver)
    # Launches Orbbec cameras + Vision Server Nodes
    vision_pkg = FindPackageShare('vision_server')
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vision_pkg, '/launch/vision_system.launch.py']),
        launch_arguments={
            'camera1_serial': LaunchConfiguration('camera1_serial'),
            'camera2_serial': LaunchConfiguration('camera2_serial'),
            'depth_camera_serial': LaunchConfiguration('depth_camera_serial'),
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

    # 3. Hand Service (single instance, namespace determined by hand_side)
    # "left"  -> namespace /lhandpro_service  (matches system_controller's "lhand" target)
    # "right" -> namespace /rhandpro_service  (matches system_controller's "rhand" target)
    hand_service_node = Node(
        package='lhandpro_service',
        executable='lhandpro_service',
        name=PythonExpression(["'lhandpro_service' if '", hand_side, "' == 'left' else 'rhandpro_service'"]),
        namespace=PythonExpression(["'/' + ('lhandpro_service' if '", hand_side, "' == 'left' else 'rhandpro_service')"]),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'ethercat_channel': LaunchConfiguration('ethercat_channel'),
            'hand_side': hand_side,
        }]
    )

    # Hand Description & State Publisher
    lhand_desc_pkg = FindPackageShare('dh116_l000_a1')
    lhand_urdf_path = PathJoinSubstitution([lhand_desc_pkg, 'urdf', 'DH116-L000-A1.urdf'])
    rhand_desc_pkg = FindPackageShare('dh116_r000_a1')
    rhand_urdf_path = PathJoinSubstitution([rhand_desc_pkg, 'urdf', 'DH116-R000-A1.urdf'])

    # Angles topic follows the namespace: /lhandpro_service/now_angles or /rhandpro_service/now_angles
    hand_angles_topic = PythonExpression([
        "'/lhandpro_service/now_angles' if '", hand_side, "' == 'left' else '/rhandpro_service/now_angles'"
    ])

    # Hand joint state publisher (publishes L_finger* or R_finger* names to /joint_states)
    hand_state_publisher = Node(
        package='lhandpro_description',
        executable='lhandpro_state_publisher',
        name='lhandpro_state_publisher',
        output='screen',
        parameters=[{
            'joint_prefix': hand_joint_prefix,
            'angles_topic': hand_angles_topic,
        }]
    )

    # 4. Lift Platform Server (RS485 Modbus RTU)
    lift_server_node = Node(
        package='lift_server',
        executable='lift_server_node',
        name='lift_server',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lift_serial_port'),
            'baud_rate': 19200,
            'slave_id': 1,
        }]
    )

    # 5. System Controller (Flow Control)
    sys_ctrl_node = Node(
        package='system_controller',
        executable='system_controller_node',
        name='system_controller',
        output='screen'
    )

    # 5.5. Glove-Hand control is now integrated into ui_app's Glove Tab
    #      (no separate glove_node or glove_hand_bridge needed)

    # 5.6. Vision Grasp (conditional — only when grasp_camera_ns is set)
    vision_grasp_pkg = FindPackageShare('vision_grasp')
    vision_grasp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vision_grasp_pkg, '/launch/vision_grasp.launch.py']),
        launch_arguments={
            'camera_ns': LaunchConfiguration('grasp_camera_ns'),
            'launch_camera': 'false',
            'calibration': LaunchConfiguration('enable_calibration'),
        }.items(),
        condition=IfCondition(enable_grasp),
    )

    # 6. UI App
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

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ui_node,
            on_exit=[Shutdown()]
        )
    )

    return LaunchDescription([
        robot_ip_arg,
        model_arg,
        ethercat_channel_arg,
        hand_side_arg,
        camera1_serial_arg,
        camera2_serial_arg,
        depth_camera_serial_arg,
        lift_serial_port_arg,
        grasp_camera_ns_arg,
        enable_calibration_arg,
        vision_launch,
        robot_launch,
        robot_state_publisher_node,
        hand_service_node,
        hand_state_publisher,
        lift_server_node,
        sys_ctrl_node,
        vision_grasp_launch,
        ui_delayed,
        exit_event_handler
    ])
