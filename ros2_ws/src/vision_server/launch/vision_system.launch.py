import os
import re
import shutil
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _detect_orbbec_cameras():
    """Auto-detect connected Orbbec cameras via list_devices_node.

    Returns a list of dicts: [{'serial': 'CV2R...', 'usb_port': '2-4'}, ...]
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

        # Parse output blocks — each camera has "serial: XXX" and "usb port: X-X"
        current = {}
        for line in output.splitlines():
            m_sn = re.search(r'\bserial:\s*(\S+)', line)
            m_port = re.search(r'\busb port:\s*(\S+)', line)
            if m_sn:
                current['serial'] = m_sn.group(1)
            if m_port:
                current['usb_port'] = m_port.group(1)
            # When we have both, save and reset
            if 'serial' in current and 'usb_port' in current:
                if current['serial'] not in [c['serial'] for c in cameras]:
                    cameras.append(dict(current))
                current = {}
    except Exception:
        pass
    return cameras


def make_camera_node(ns, serial_number, device_num=1, usb_port='', connection_delay=100):
    """Create a ComposableNodeContainer for one Orbbec camera.

    ns: the ROS namespace AND camera_name — equals the SN when known.
    """
    params = {
        'camera_name': ns,
        'serial_number': serial_number,
        'device_num': device_num,
        'depth_registration': True,
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

    # Auto-detect camera serial numbers and USB ports when not explicitly provided
    detected = _detect_orbbec_cameras()
    cam1_port = ''
    cam2_port = ''

    if not camera1_sn and len(detected) >= 1:
        camera1_sn = detected[0]['serial']
        cam1_port = detected[0].get('usb_port', '')
    elif camera1_sn:
        # Find USB port for explicitly provided SN
        for d in detected:
            if d['serial'] == camera1_sn:
                cam1_port = d.get('usb_port', '')
                break

    if not camera2_sn and len(detected) >= 2:
        camera2_sn = detected[1]['serial']
        cam2_port = detected[1].get('usb_port', '')
    elif camera2_sn:
        for d in detected:
            if d['serial'] == camera2_sn:
                cam2_port = d.get('usb_port', '')
                break

    actions = []
    num_cameras = 2 if camera2_sn else 1

    if camera2_sn:
        # Dual camera: stagger launches by 2s (following Orbbec multi_camera example)
        ns1 = camera1_sn if camera1_sn else 'camera_1'
        ns2 = camera2_sn
        actions.append(TimerAction(
            period=0.0,
            actions=[make_camera_node(ns1, camera1_sn, device_num=num_cameras,
                                      usb_port=cam1_port, connection_delay=100)],
        ))
        actions.append(TimerAction(
            period=2.0,
            actions=[make_camera_node(ns2, camera2_sn, device_num=num_cameras,
                                      usb_port=cam2_port, connection_delay=100)],
        ))
    else:
        # Single camera
        ns1 = camera1_sn if camera1_sn else 'camera'
        actions.append(make_camera_node(ns1, camera1_sn, device_num=num_cameras,
                                        usb_port=cam1_port))

    # image_saver_node
    actions.append(Node(
        package='vision_server',
        executable='image_saver_node',
        name='image_saver_node',
        namespace='image_saver',
        output='screen',
        parameters=[{'save_dir': '.'}]
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera1_serial',
            default_value='',
            description='Serial number of camera 1 (e.g. CV2R1610004Y). Empty = auto-detect.'
        ),
        DeclareLaunchArgument(
            'camera2_serial',
            default_value='',
            description='Serial number of camera 2 (e.g. CV2R1610004H). Empty = auto-detect.'
        ),
        OpaqueFunction(function=generate_actions),
    ])
