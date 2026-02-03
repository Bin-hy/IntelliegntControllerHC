import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions

def generate_launch_description():
    # 获取默认urdf路径
    urdf_package_path = get_package_share_directory('lhandpro_description')
    # Right hand: DH126S-R000-A1.urdf,  Left hand: DH126S-L000-A1.urdf
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'DH126S-R000-A1.urdf')
    default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    # 声明urdf目录参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(name='model', default_value=default_urdf_path, description='加载模型路径')
    # 通过文件路径获取内容并转换成参数值对象， 以传入 robot_state_publisher
    substitutions_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type=str)

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}],
        output='screen',
        emulate_tty=True
    )

    action_lhandpro_state_publisher = launch_ros.actions.Node(
        package='lhandpro_description',         # 包名
        executable='lhandpro_state_publisher',  # 可执行文件名
        name='lhandpro_state_publisher', # 节点名称
        output='screen',                 # 输出日志到终端
        emulate_tty=True                 # 更好地显示日志
    )

    action_joint_state_publisher_gui = launch_ros.actions.Node(
        package='joint_state_publisher_gui',      # 包名
        executable='joint_state_publisher_gui',   # 可执行文件名
        name='joint_state_publisher_gui',         # 节点名称 (可选，可以自定义)
        output='screen',                          # 输出日志到终端
        emulate_tty=True,                         # 更好地显示日志
    )

    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_lhandpro_state_publisher,
        # action_joint_state_publisher_gui,
        action_rviz_node
    ])