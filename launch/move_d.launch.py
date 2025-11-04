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
    node_move_analyte_to_ready = Node(
        package="titration_robot_cell_moveit_control",
        executable="move_analyte_to_ready",
        output="screen",
        parameters=[
            # {"use_sim_time": True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    node_move_ready_to_home = Node(
        package="titration_robot_cell_moveit_control",
        executable="move_ready_to_home",
        output="screen",
        parameters=[
            # {"use_sim_time": True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    on_exit_hander = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_move_analyte_to_ready,
            on_exit=[node_move_ready_to_home],
        )
    )

    return LaunchDescription([
        node_move_analyte_to_ready,
        on_exit_hander,
    ])