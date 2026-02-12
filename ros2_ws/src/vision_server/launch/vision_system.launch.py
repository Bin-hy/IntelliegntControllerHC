import re
import subprocess
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def get_connected_devices():
    devices = []
    try:
        # Run the list_devices_node to get connected Orbbec cameras
        cmd = ['ros2', 'run', 'orbbec_camera', 'list_devices_node']
        # Set environment to ensure ROS2 commands work if not already set (though usually it is)
        result = subprocess.run(cmd, capture_output=True, text=True)
        output = result.stdout + result.stderr
        
        print(f"[DEBUG] Device List Output:\n{output}") # Print to stdout for debugging
        
        # Regex to extract device info
        # We look for blocks of info. The output format seems to be line-by-line logging.
        # Strategy: Find all "PID: 0x..." lines, then find the associated "usb port:" line.
        
        current_device = {}
        for line in output.splitlines():
            # Match PID and SN
            # Example: ... - Name: Orbbec Gemini 335, PID: 0x0800, SN/ID: CP0HC530000F, Connection: USB3.2
            # Also handle cases where SN might be labelled differently or missing
            pid_match = re.search(r'PID: 0x([0-9a-fA-F]+)', line)
            if pid_match:
                if current_device:
                    devices.append(current_device)
                current_device = {
                    'pid': pid_match.group(1),
                    'sn': 'unknown',
                    'port': None,
                    'name': 'Unknown'
                }
                
                # Try to extract SN
                sn_match = re.search(r'SN/ID: ([A-Za-z0-9]+)', line)
                if sn_match:
                    current_device['sn'] = sn_match.group(1)
                else:
                    # Fallback regex for serial
                    sn_match_2 = re.search(r'serial: ([A-Za-z0-9]+)', line)
                    if sn_match_2:
                        current_device['sn'] = sn_match_2.group(1)

                # Try to extract name
                name_match = re.search(r'Name: (.*?), PID:', line)
                if name_match:
                    current_device['name'] = name_match.group(1)
                continue
            
            # Additional check for SN if it appears on a separate line (common in some logs)
            if current_device and current_device['sn'] == 'unknown':
                 sn_match_line = re.search(r'serial: ([A-Za-z0-9]+)', line)
                 if sn_match_line:
                     current_device['sn'] = sn_match_line.group(1)
                
            # Match USB Port
            # Example: ... usb port: 2-2.1
            port_match = re.search(r'usb port: ([0-9.-]+)', line)
            if port_match and current_device:
                current_device['port'] = port_match.group(1)
        
        if current_device:
            devices.append(current_device)
            
    except Exception as e:
        print(f"Error detecting devices: {e}")
        
    return devices

def launch_setup(context, *args, **kwargs):
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    actions = []
    
    devices = get_connected_devices()
    
    if not devices:
        actions.append(LogInfo(msg="[Vision System] No Orbbec devices detected!"))
        return actions
        
    actions.append(LogInfo(msg=f"[Vision System] Detected {len(devices)} devices."))
    
    for i, dev in enumerate(devices):
        pid = dev.get('pid', '')
        sn = dev.get('sn', f'unknown_{i}')
        port = dev.get('port', '')
        name = dev.get('name', 'Camera')
        
        if not port:
            actions.append(LogInfo(msg=f"[Vision System] Skipping device {sn} (No port found)"))
            continue
            
        # Determine Launch File based on Device Name (More robust than PID)
        launch_file = 'gemini_330_series.launch.py' # Default fallback
        
        name_lower = name.lower()
        if 'gemini 305' in name_lower:
            launch_file = 'gemini305.launch.py'
        elif 'gemini 33' in name_lower: # 330, 335, 336...
            launch_file = 'gemini_330_series.launch.py'
        elif 'gemini 2 l' in name_lower:
            launch_file = 'gemini2L.launch.py'
        elif 'gemini 2' in name_lower:
            launch_file = 'gemini2.launch.py'
        elif 'astra' in name_lower:
            launch_file = 'astra.launch.py'
        elif 'femto bolt' in name_lower:
            launch_file = 'femto_bolt.launch.py'
        elif 'femto mega' in name_lower:
            launch_file = 'femto_mega.launch.py'
        elif 'femto' in name_lower:
            launch_file = 'femto.launch.py'
        elif 'dabai' in name_lower:
            launch_file = 'dabai.launch.py' # Assuming dabai exists, otherwise fallback or check specific dabai launch
        
        # Create a unique namespace based on Serial Number
        if sn == 'unknown':
             # Generate a random suffix if SN is missing to avoid conflicts
             import random
             sn = f"unknown_{random.randint(1000, 9999)}"
             
        camera_ns = f"camera_{sn}"
        
        actions.append(LogInfo(msg=f"[Vision System] Launching {name} (PID: {pid}) at {port} as {camera_ns} using {launch_file}"))
        
        # Common parameters for all cameras
        # Note: Some older launch files might not support 'enable_left_ir'/'enable_right_ir' and use 'enable_ir' instead.
        # But most recent Orbbec ROS2 launch files support specific IR control.
        # We will try to provide a superset of params. Launch system usually ignores unknown args unless declared strictly.
        
        launch_args = {
            'camera_name': camera_ns,
            'usb_port': port,
            'enable_color': 'true',
            'enable_depth': 'true',
            'depth_registration': 'false', # Disable to save resources/stability
            'enable_point_cloud': 'false', # Disable for stability
            'enable_disparity_to_depth': 'false', # Fix for 'depth frame processor' error
            'enable_d2c_viewer': 'false'
        }
        
        # Specific parameter adjustments
        if 'gemini305' in launch_file:
             launch_args.update({
                 'enable_left_ir': 'false',  # Disable to save bandwidth on USB 2.0
                 'enable_right_ir': 'false', # Disable to save bandwidth on USB 2.0
                 'enable_depth': 'true',     # Keep depth enabled
                 'enable_color': 'true'
             })
        elif 'gemini_330' in launch_file:
             launch_args.update({
                 'enable_left_ir': 'true',
                 'enable_right_ir': 'true'
             })
        else:
             # For other cameras, try generic IR enable if needed, or stick to color/depth
             # Many cameras like Astra only have one IR or use different params.
             # Safest is to enable color/depth by default.
             pass

        # Use TimerAction to stagger launches by 3 seconds per camera to avoid USB bandwidth spikes
        launch_delay = 3.0 * i
        
        launch_action = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(orbbec_camera_dir, 'launch', launch_file)]),
                launch_arguments=launch_args.items()
            )
        ])
        
        if launch_delay > 0:
             actions.append(LogInfo(msg=f"[Vision System] Delaying launch of {camera_ns} by {launch_delay}s"))
             actions.append(TimerAction(period=launch_delay, actions=[launch_action]))
        else:
             actions.append(launch_action)
             
    return actions
        
    return actions

def generate_launch_description():
    # Image Saver Node
    # Handles saving images from any camera topic via service call
    saver_node = Node(
        package='vision_server',
        executable='image_saver_node',
        name='image_saver',
        parameters=[{
            'save_dir': '/home/user/IntelliegntControllerHC/photos',
            'service_name': 'save_image'
        }]
    )

    return LaunchDescription([
        LogInfo(msg="Launching Vision System with Dynamic Camera Discovery..."),
        saver_node,
        OpaqueFunction(function=launch_setup)
    ])
