from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch



#def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("dual_arms", package_name="duco_arms").to_moveit_configs()
#    return generate_demo_launch(moveit_config)
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from launch_ros.actions import Node



from moveit_configs_utils import MoveItConfigsBuilder
#from moveit_configs_utils.launches import generate_demo_launch

def generate_demo_launch(moveit_config, launch_package_path=None):
    """
    Launches a self contained demo

    launch_package_path is optional to use different launch and config packages

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    if launch_package_path == None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )
    ld.add_action(
    	DeclareLaunchArgument(
    	'arm_num', 
    	default_value='2',
    	)
    )
    ld.add_action(
    	DeclareLaunchArgument(
    	'arm_dof', 
    	default_value='7',
    	)
    )
    ld.add_action(
    	DeclareLaunchArgument(
    	'server_host_1', 
    	default_value='127.0.0.1',
    	)
    )
    ld.add_action(
    	DeclareLaunchArgument(
    	'server_host_2', 
    	default_value='127.0.0.1',
    	)
    )
    # Fake joint driver
    #ld.add_action(
    #    Node(
    #        package="controller_manager",
    #        executable="ros2_control_node",
    #        parameters=[
    #            moveit_config.robot_description,
    #            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
    #        ],
    #    )
    #)
        # Fake joint driver
    ld.add_action(
        Node(
            package="duco_ros_driver",
            executable="DucoDriver",
            parameters=[{'arm_num':LaunchConfiguration('arm_num')},{'server_host_1':LaunchConfiguration('server_host_1')},{'server_host_2':LaunchConfiguration('server_host_2')}]
        )
    )
    ld.add_action(
        Node(
            package="duco_ros_driver",
            executable="DucoRobotStatus",
            parameters=[{'arm_num':LaunchConfiguration('arm_num')},{"arm_dof":LaunchConfiguration("arm_dof")},{'server_host_1':LaunchConfiguration('server_host_1')},{'server_host_2':LaunchConfiguration('server_host_2')}]
        )
    )
    ld.add_action(
        Node(
            package="duco_ros_driver",
            executable="DucoTrajectoryAction",
            parameters=[{'arm_num':LaunchConfiguration('arm_num')},{'server_host_1':LaunchConfiguration('server_host_1')},{'server_host_2':LaunchConfiguration('server_host_2')}]
        )
    )
    ld.add_action(
        Node(
            package="duco_ros_driver",
            executable="DucoRobotControl",
            parameters=[{'arm_num':LaunchConfiguration('arm_num')},{"arm_dof":LaunchConfiguration("arm_dof")},{'server_host_1':LaunchConfiguration('server_host_1')},{'server_host_2':LaunchConfiguration('server_host_2')}]
        )
    )

    return ld
    
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("double_arm", package_name="double_arm_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
