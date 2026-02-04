from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    orbbec_camera_pkg = FindPackageShare('orbbec_camera')
    
    # Orbbec Camera Launch
    # We enable depth registration to align depth to color, which is often useful for vision tasks
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([orbbec_camera_pkg, '/launch/gemini305.launch.py']),
        launch_arguments={
            'camera_name': 'camera',
            'depth_registration': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_ir': 'true'
        }.items()
    )
    
    # Image Saver Node
    saver_node = Node(
        package='vision_server',
        executable='image_saver_node',
        name='image_saver',
        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'save_dir': '/home/user/IntelliegntControllerHC/photos',
            'service_name': 'save_image'
        }]
    )
    
    return LaunchDescription([
        camera_launch,
        saver_node
    ])
