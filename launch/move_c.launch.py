from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    RegisterEventHandler
)
from launch.event_handlers import (
    OnProcessExit
)
from moveit_configs_utils import MoveItConfigsBuilder
# import launch_ros.actions

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("titration_robot_cell").to_moveit_configs()
    # launch_ros.actions.SetParameter(name='use_sim_time', value=True)
    node_move_analyte_to_titrant = Node(
        package="titration_robot_cell_moveit_control",
        executable="move_analyte_to_titrant",
        output="screen",
        parameters=[
            # {"use_sim_time": True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        node_move_analyte_to_titrant,
    ])