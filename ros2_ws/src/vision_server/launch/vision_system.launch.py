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
    # Gemini 215 / 210 series (PID 0x0808 / 0x0809) — use gemini210 launch profile
    if any(tag in name for tag in ['215', '210']):
        return 'gemini210'
    # Gemini 2 / 2L / 2XL series (PID 0x0670 / 0x0673)
    if any(tag in name for tag in ['gemini 2', 'gemini2', '2l']):
        return 'gemini2l'
    return 'default'


def make_camera_node(ns, serial_number, device_num=1, usb_port='', connection_delay=100,
                     camera_profile='default'):
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
                'enable_colored_point_cloud': 'false',
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_ir': 'false',
                'log_level': 'none',
            }.items()
        )

    if camera_profile == 'gemini210':
        # Gemini 210/215 (PID 0x0808/0x0809) — build ComposableNode directly
        # so we can set enable_sync_host_time=false (timerSyncWithHost crashes in SDK)
        params = {
            'camera_name': ns,
            'serial_number': serial_number,
            'device_num': device_num,
            'uvc_backend': 'libuvc',
            'depth_registration': True,
            'enable_point_cloud': True,
            'enable_colored_point_cloud': False,
            'enable_d2c_viewer': False,
            'enable_frame_sync': True,
            'sync_mode': 'standalone',
            'enable_color': True,
            'enable_depth': True,
            'enable_ir': False,
            'enable_accel': False,
            'enable_gyro': False,
            'connection_delay': connection_delay,
            'log_level': 'none',
            'color_width': 0,
            'color_height': 0,
            'color_fps': 0,
            'color_format': 'ANY',
            'depth_width': 0,
            'depth_height': 0,
            'depth_fps': 0,
            'depth_format': 'ANY',
            'ir_width': 0,
            'ir_height': 0,
            'ir_fps': 0,
            'ir_format': 'ANY',
            'disparity_to_depth_mode': 'SW',
            'align_mode': 'HW',
            'enable_depth_scale': True,
            'enable_ldp': True,
            'enable_heartbeat': False,
            'enable_noise_removal_filter': True,
            'noise_removal_filter_min_diff': 256,
            'noise_removal_filter_max_size': 200,
            'enable_spatial_filter': True,
            'spatial_filter_magnitude': 2,
            'spatial_filter_alpha': 0.5,
            'spatial_filter_diff_threshold': 20,
            'enable_temporal_filter': True,
            'temporal_filter_diff_threshold': 20.0,
            'temporal_filter_weight': 0.4,
            'enable_threshold_filter': True,
            'threshold_filter_min': 100,       # 100mm 最小深度
            'threshold_filter_max': 5000,      # 5000mm 最大深度
            'enable_sync_host_time': False,     # SDK timerSyncWithHost crashes for Gemini 215
            'time_domain': 'global',
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

    # Default (Gemini 330 series)
    params = {
        'camera_name': ns,
        'serial_number': serial_number,
        'device_num': device_num,
        'depth_registration': True,
        'align_mode': 'SW',              # SW alignment — HW fails with auto-detect profiles
        'enable_point_cloud': True,
        'enable_colored_point_cloud': True,
        'enable_d2c_viewer': False,
        'enable_frame_sync': True,
        'sync_mode': 'standalone',
        'enable_color': True,
        'enable_depth': True,
        'enable_left_ir': False,
        'enable_right_ir': False,
        'device_preset': 'Default',
        'connection_delay': connection_delay,
        'log_level': 'none',
        'color_width': 0,
        'color_height': 0,
        'color_fps': 0,
        'color_format': 'ANY',
        'depth_width': 0,
        'depth_height': 0,
        'depth_fps': 0,
        'depth_format': 'ANY',
        'ir_width': 0,
        'ir_height': 0,
        'ir_fps': 0,
        'ir_format': 'ANY',
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


def generate_actions(context):
    camera1_sn = LaunchConfiguration('camera1_serial').perform(context)
    camera2_sn = LaunchConfiguration('camera2_serial').perform(context)
    depth_cam_sn = LaunchConfiguration('depth_camera_serial').perform(context)

    # Auto-detect camera serial numbers and USB ports when not explicitly provided
    detected = _detect_orbbec_cameras()

    # Build lookup tables by serial number
    info_map = {d['serial']: d for d in detected}
    port_map = {d['serial']: d.get('usb_port', '') for d in detected}

    # Assign explicitly-specified cameras first, collecting used SNs
    used_sns = set()

    if camera1_sn:
        used_sns.add(camera1_sn)
    if camera2_sn:
        used_sns.add(camera2_sn)
    if depth_cam_sn:
        used_sns.add(depth_cam_sn)

    # Auto-detect remaining cameras from unassigned detected devices
    remaining = [d for d in detected if d['serial'] not in used_sns]

    if not camera1_sn and remaining:
        camera1_sn = remaining.pop(0)['serial']
        used_sns.add(camera1_sn)

    if not camera2_sn and remaining:
        camera2_sn = remaining.pop(0)['serial']
        used_sns.add(camera2_sn)

    if not depth_cam_sn and remaining:
        depth_cam_sn = remaining.pop(0)['serial']

    actions = []
    # Total camera count for device_num hint
    cam_count = sum(1 for sn in [camera1_sn, camera2_sn, depth_cam_sn] if sn)

    # --- Launch cameras with staggered delays (following Orbbec multi_camera pattern) ---
    # Each camera needs ~5s to fully initialize and release the device lock.
    ns1 = camera1_sn if camera1_sn else 'camera'
    cam1_profile = _camera_profile(info_map.get(camera1_sn, {})) if camera1_sn else 'default'
    if camera2_sn:
        ns2 = camera2_sn
        cam2_profile = _camera_profile(info_map.get(camera2_sn, {}))
        actions.append(TimerAction(
            period=0.0,
            actions=[GroupAction([make_camera_node(ns1, camera1_sn, device_num=cam_count,
                                      usb_port=port_map.get(camera1_sn, ''),
                                      connection_delay=100, camera_profile=cam1_profile)])],
        ))
        actions.append(TimerAction(
            period=6.0,
            actions=[GroupAction([make_camera_node(ns2, camera2_sn, device_num=cam_count,
                                      usb_port=port_map.get(camera2_sn, ''),
                                      connection_delay=100, camera_profile=cam2_profile)])],
        ))
    else:
        actions.append(make_camera_node(ns1, camera1_sn, device_num=cam_count,
                                        usb_port=port_map.get(camera1_sn, ''),
                                        camera_profile=cam1_profile))

    # --- Launch depth camera (3rd camera, e.g. Gemini 215) ---
    depth_ns = ''
    if depth_cam_sn:
        depth_ns = depth_cam_sn
        depth_profile = _camera_profile(info_map.get(depth_cam_sn, {}))
        delay = 12.0 if camera2_sn else 6.0
        actions.append(TimerAction(
            period=delay,
            actions=[GroupAction([make_camera_node(depth_ns, depth_cam_sn, device_num=cam_count,
                                      usb_port=port_map.get(depth_cam_sn, ''),
                                      connection_delay=100, camera_profile=depth_profile)])],
        ))

    # image_saver_node
    actions.append(Node(
        package='vision_server',
        executable='image_saver_node',
        name='image_saver_node',
        namespace='image_saver',
        output='screen',
        parameters=[{'save_dir': os.path.expanduser('~/.ros/task_photos')}]
    ))

    # depth_measure_node — uses depth camera if available, else camera 1
    depth_measure_ns = '/' + depth_ns if depth_ns else ('/' + ns1 if ns1 else '/camera')
    actions.append(Node(
        package='vision_server',
        executable='depth_measure_node',
        name='depth_measure_node',
        namespace='depth_measure',
        output='screen',
        parameters=[{
            'camera_ns': depth_measure_ns,
            'roi_width': 200,
            'roi_height': 200,
            'roi_center_u': -1,        # -1 = image center
            'roi_center_v': -1,
            'min_depth_mm': 100.0,
            'max_depth_mm': 5000.0,
            'cluster_range_mm': 50.0,
            'publish_hz': 10.0,
            'save_dir': os.path.expanduser('~/.ros/depth_measure'),
        }]
    ))

    # earphone_inspector_node — uses depth camera (215) if available, else camera 1
    inspector_ns = '/' + depth_ns if depth_ns else ('/' + ns1 if ns1 else '/camera')
    actions.append(Node(
        package='vision_server',
        executable='earphone_inspector_node',
        name='earphone_inspector_node',
        namespace='earphone_inspector',
        output='screen',
        parameters=[{
            'camera_ns': inspector_ns,
            'min_diff_mm': 3.0,
            'max_diff_mm': 80.0,
            'min_area_pixels': 50,
            'roi_width': 300,
            'roi_height': 300,
            'save_dir': os.path.expanduser('~/.ros/earphone_inspection'),
        }]
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera1_serial',
            default_value='',
            description='Serial number of camera 1 (e.g. CP0HC530000F). Empty = auto-detect.'
        ),
        DeclareLaunchArgument(
            'camera2_serial',
            default_value='',
            description='Serial number of camera 2 (e.g. CV2R1610004H). Empty = auto-detect.'
        ),
        DeclareLaunchArgument(
            'depth_camera_serial',
            default_value='',
            description='Serial number of depth measurement camera (e.g. AYZ8953002L / Gemini 215). Empty = auto-detect 3rd camera.'
        ),
        OpaqueFunction(function=generate_actions),
    ])
