from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("gcr20_1400", package_name="duco_gcr20_1400_moveit_config").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
