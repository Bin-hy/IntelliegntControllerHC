import os
import re
import shutil
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def _detect_orbbec_cameras():
    """Auto-detect connected Orbbec cameras via list_devices_node.

    Returns a list of dicts: [{'serial': 'CV2R...', 'usb_port': '2-4', 'name': 'Orbbec Gemini 335'}, ...]
    """
    cameras = []
    try:
        candidates = []
        # Derive the install root from this launch file's location
        # __file__ = .../install/vision_server/share/vision_server/launch/vision_system.launch.py
        this_dir = os.path.dirname(os.path.abspath(__file__))
        # Walk up to find the install root (contains multiple package prefixes)
        install_root = this_dir
        for _ in range(4):  # launch -> vision_server -> share -> vision_server -> install
            install_root = os.path.dirname(install_root)
        candidates.append(os.path.join(install_root, 'orbbec_camera', 'lib', 'orbbec_camera', 'list_devices_node'))
        ros2_ws = os.path.expanduser('~/IntelliegntControllerHC/ros2_ws')
        candidates.append(os.path.join(ros2_ws, 'install', 'orbbec_camera', 'lib', 'orbbec_camera', 'list_devices_node'))
        candidates.append(os.path.join(ros2_ws, 'build', 'orbbec_camera', 'list_devices_node'))

        exe = None
        for c in candidates:
            if os.path.isfile(c) and os.access(c, os.X_OK):
                exe = c
                break
        if exe is None:
            exe = shutil.which('list_devices_node')
        if exe is None:
            return cameras

        result = subprocess.run([exe], capture_output=True, text=True, timeout=5)
        output = result.stdout + result.stderr

        # Parse output blocks — each camera has "Name: XXX", "serial: XXX" and "usb port: X-X"
        current = {}
        for line in output.splitlines():
            m_name = re.search(r'Name:\s*(.+?),\s*PID:', line)
            m_sn = re.search(r'\bserial:\s*(\S+)', line)
            m_port = re.search(r'\busb port:\s*(\S+)', line)
            if m_name:
                current['name'] = m_name.group(1).strip()
            if m_sn:
                current['serial'] = m_sn.group(1)
            if m_port:
                current['usb_port'] = m_port.group(1)
            # When we have both serial and port, save and reset
            if 'serial' in current and 'usb_port' in current:
                if current['serial'] not in [c['serial'] for c in cameras]:
                    cameras.append(dict(current))
                current = {}
    except Exception:
        pass
    return cameras


def _camera_profile(cam_info):
    """Determine launch profile based on detected camera model name."""
    name = cam_info.get('name', '').lower()
    # Gemini 215 / 210 series (PID 0x0808 / 0x0809) — use gemini2.launch.py per vendor
    if any(tag in name for tag in ['215', '210']):
        return 'gemini2'
    # Gemini 2 / 2L / 2XL series (PID 0x0670 / 0x0673)
    if any(tag in name for tag in ['gemini 2', 'gemini2', '2l']):
        return 'gemini2l'
    return 'default'


def _sn_to_ns(serial_number, usb_port=''):
    """Return a valid ROS 2 namespace for a camera.
    Valid SNs (start with letter) are used as-is.
    Invalid SNs (start with digit) fall back to a port-based name like port_2_2.
    """
    if serial_number and not serial_number[0].isdigit():
        return serial_number
    if usb_port:
        return 'port_' + usb_port.replace('-', '_')
    return 'camera_unknown'


def make_camera_node(ns, serial_number, device_num=1, usb_port='', connection_delay=100,
                     camera_profile='default', enable_depth=True):
    """Create a ComposableNodeContainer for one Orbbec camera.

    ns: the ROS namespace AND camera_name — equals the SN when known.
    camera_profile: 'default' for 330-series, 'gemini210' for 210/215, 'gemini2l' for Gemini 2/2L.
    """
    if camera_profile == 'gemini2l':
        # Gemini 2/2L → gemini2L.launch.py (PID 0x0670/0x0673)
        orbbec_share = get_package_share_directory('orbbec_camera')
        launch_file = os.path.join(orbbec_share, 'launch', 'gemini2L.launch.py')
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'camera_name': ns,
                'serial_number': serial_number,
                'device_num': str(device_num),
                'usb_port': usb_port or '',
                'connection_delay': str(connection_delay),
                'enable_point_cloud': 'true',
                'enable_colored_point_cloud': 'true',
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_ir': 'true',
                'log_level': 'none',
            }.items()
        )

    if camera_profile in ('gemini2', 'gemini210'):
        # Gemini 215/210 → gemini2.launch.py (per vendor recommendation)
        orbbec_share = get_package_share_directory('orbbec_camera')
        launch_file = os.path.join(orbbec_share, 'launch', 'gemini2.launch.py')
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'camera_name': ns,
                'serial_number': serial_number,
                'device_num': str(device_num),
                'usb_port': usb_port or '',
                'connection_delay': str(connection_delay),
                'enable_point_cloud': 'true',
                'enable_colored_point_cloud': 'true',
                'enable_color': 'true',
                'enable_depth': str(enable_depth).lower(),
                'enable_ir': 'True',
                'log_level': 'debug',
            }.items()
        )

    # Default (Gemini 330/335 series)
    # NOTE: When IR is enabled, depth and IR MUST have the same resolution and fps.
    # Using 640x400@30fps for depth+IR to avoid mismatch errors with SDK v2.7.6.
    params = {
        'camera_name': ns,
        'serial_number': serial_number,
        'device_num': device_num,
        'depth_registration': True,
        'align_mode': 'SW',
        'enable_point_cloud': True,
        'enable_colored_point_cloud': True,
        'enable_d2c_viewer': False,
        'enable_frame_sync': True,
        'sync_mode': 'standalone',
        'enable_color': True,
        'enable_depth': True,
        'enable_left_ir': True,
        'enable_right_ir': True,
        'device_preset': 'Default',
        'connection_delay': connection_delay,
        'log_level': 'none',
        'color_width': 0,
        'color_height': 0,
        'color_fps': 0,
        'color_format': 'ANY',
        # Depth and IR must match — explicitly set to 640x400@30fps
        'depth_width': 640,
        'depth_height': 400,
        'depth_fps': 30,
        'depth_format': 'ANY',
        'left_ir_width': 640,
        'left_ir_height': 400,
        'left_ir_fps': 30,
        'left_ir_format': 'ANY',
        'right_ir_width': 640,
        'right_ir_height': 400,
        'right_ir_fps': 30,
        'right_ir_format': 'ANY',
    }
    if usb_port:
        params['usb_port'] = usb_port

    return GroupAction([
        PushRosNamespace(ns),
        ComposableNodeContainer(
            name='camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='orbbec_camera',
                    plugin='orbbec_camera::OBCameraNodeDriver',
                    name=ns,
                    parameters=[params],
                ),
            ],
            output='log',
        )
    ])


def _is_gemini_2xx(cam_info):
    """Check if camera is a Gemini 210/215 series (dedicated depth camera)."""
    name = cam_info.get('name', '').lower()
    return any(tag in name for tag in ['215', '210'])


def generate_actions(context):
    """Launch all detected cameras dynamically — no fixed camera count assumption.

    Strategy:
      - Detect all connected Orbbec cameras
      - Classify each: 210/215 → depth inspection, 330/335 → general purpose
      - Launch ALL of them with staggered delays (8s apart)
      - Pick the first 215/210 as the earphone inspection camera (depth_ns)
      - Pick the first 330/335 as the main camera (main_ns)
      - For numeric SNs (e.g. "01234567890"), use usb_port instead of SN
    """
    detected = _detect_orbbec_cameras()

    if not detected:
        import logging
        logging.getLogger('vision_system.launch').warning('No Orbbec cameras detected!')

    # Build lookup tables
    info_map = {d['serial']: d for d in detected}
    port_map = {d['serial']: d.get('usb_port', '') for d in detected}

    def _effective_sn(sn):
        """For numeric SNs, clear to force SDK to use usb_port."""
        if sn and sn[0].isdigit():
            return ''
        return sn

    # Classify cameras
    depth_cameras = []   # 210/215 series
    general_cameras = [] # 330/335/other series
    for d in detected:
        ns = _sn_to_ns(d['serial'], d.get('usb_port', ''))
        profile = _camera_profile(d)
        entry = {
            'serial': d['serial'],
            'usb_port': d.get('usb_port', ''),
            'name': d.get('name', ''),
            'ns': ns,
            'profile': profile,
        }
        if _is_gemini_2xx(d):
            depth_cameras.append(entry)
        else:
            general_cameras.append(entry)

    # Log assignment
    import logging
    logger = logging.getLogger('vision_system.launch')
    logger.info(f'Detected {len(detected)} cameras: '
                f'{len(general_cameras)} general ({[c["ns"] for c in general_cameras]}), '
                f'{len(depth_cameras)} depth ({[c["ns"] for c in depth_cameras]})')

    actions = []
    cam_count = len(detected)
    cam_index = 0  # for staggered delay

    # --- Launch all general cameras (330/335) ---
    for cam in general_cameras:
        delay = cam_index * 8.0
        actions.append(TimerAction(
            period=delay,
            actions=[GroupAction([make_camera_node(
                ns=cam['ns'],
                serial_number=_effective_sn(cam['serial']),
                device_num=cam_count,
                usb_port=cam['usb_port'],
                connection_delay=500,
                camera_profile=cam['profile'],
                enable_depth=True,
            )])],
        ))
        cam_index += 1

    # --- Launch all depth cameras (210/215) ---
    for cam in depth_cameras:
        delay = cam_index * 8.0
        actions.append(TimerAction(
            period=delay,
            actions=[GroupAction([make_camera_node(
                ns=cam['ns'],
                serial_number=_effective_sn(cam['serial']),
                device_num=cam_count,
                usb_port=cam['usb_port'],
                connection_delay=500,
                camera_profile=cam['profile'],
                enable_depth=True,
            )])],
        ))
        cam_index += 1

    # --- Determine namespaces for vision nodes ---
    # depth_ns: prefer first 215/210 camera for earphone inspection
    # main_ns: prefer first 330/335 camera for general use
    depth_ns = depth_cameras[0]['ns'] if depth_cameras else ''
    main_ns = general_cameras[0]['ns'] if general_cameras else ''
    fallback_ns = depth_ns or main_ns or 'camera'

    # image_saver_node
    actions.append(Node(
        package='vision_server',
        executable='image_saver_node',
        name='image_saver_node',
        namespace='image_saver',
        output='screen',
        parameters=[{'save_dir': os.path.expanduser('~/.ros/task_photos')}]
    ))

    # depth_measure_node — uses depth camera if available, else main camera
    depth_measure_cam = '/' + (depth_ns or fallback_ns)
    actions.append(Node(
        package='vision_server',
        executable='depth_measure_node',
        name='depth_measure_node',
        namespace='depth_measure',
        output='screen',
        parameters=[{
            'camera_ns': depth_measure_cam,
            'roi_width': 200,
            'roi_height': 200,
            'roi_center_u': -1,
            'roi_center_v': -1,
            'min_depth_mm': 100.0,
            'max_depth_mm': 5000.0,
            'cluster_range_mm': 50.0,
            'publish_hz': 10.0,
            'save_dir': os.path.expanduser('~/.ros/depth_measure'),
        }]
    ))

    # earphone_inspector_node — uses depth camera (215) if available, else main
    inspector_cam = '/' + (depth_ns or fallback_ns)
    actions.append(Node(
        package='vision_server',
        executable='earphone_inspector_node',
        name='earphone_inspector_node',
        namespace='earphone_inspector',
        output='screen',
        parameters=[{
            'camera_ns': inspector_cam,
            'avg_frames': 8,
            'min_diff_mm': 2.0,
            'max_diff_mm': 50.0,
            'min_area_pixels': 30,
            'roi_width': 250,
            'roi_height': 250,
            'plane_inlier_mm': 3.0,
            'plane_ransac_iters': 50,
            'plane_fg_min_mm': 2.0,
            'plane_fg_max_mm': 50.0,
            'pca_trim_pct': 0.10,
            'noise_sigma_scale': 2.5,
            'enable_aruco': True,
            'aruco_marker_size_m': 0.03,
            'enable_multi_trial': True,
            'min_confidence': 0.35,
            'save_dir': os.path.expanduser('~/.ros/earphone_inspection'),
        }]
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=generate_actions),
    ])
