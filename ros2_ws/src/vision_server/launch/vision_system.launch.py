from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Args
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera namespace'
    )
    
    # Orbbec Camera Launch
    # We use the standard launch file provided by orbbec_camera package
    # and override parameters for optimal performance in this system
    orbbec_pkg = FindPackageShare('orbbec_camera')
    
    # Define launch arguments for the camera
    camera_launch_args = {
        'camera_name': LaunchConfiguration('camera_name'),
        'depth_registration': 'true',      # Enable Depth-Color alignment
        'enable_point_cloud': 'false',     # Disable point cloud to save bandwidth (we use depth map)
        'enable_colored_point_cloud': 'false',
        'enable_d2c_viewer': 'false',
        'enable_frame_sync': 'true',       # Enable frame sync
        'sync_mode': 'standalone',         # Standalone mode
        'enable_color': 'true',
        'enable_depth': 'true',
        'device_preset': 'Default',
    }

    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([orbbec_pkg, '/launch/dabai_al.launch.py']), # Assuming Gemini 330/dabai series
        launch_arguments=camera_launch_args.items()
    )
    
    # Image Saver Node
    # This node provides the SaveImage service
    image_saver_node = Node(
        package='vision_server',
        executable='image_saver_node',
        name='image_saver_node',
        namespace='image_saver',
        output='screen'
    )

    return LaunchDescription([
        camera_name_arg,
        orbbec_launch,
        image_saver_node
    ])
