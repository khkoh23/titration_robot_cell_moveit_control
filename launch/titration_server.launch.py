from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pipette_server_node = Node(
        package='titration_robot_cell_moveit_control',
        executable='pipette_server',
        output='screen',
    )

    move_pose_server_node = Node(
        package='titration_robot_cell_moveit_control',
        executable='move_pose_server',
        output='screen',
    )

    move_state_server_node = Node(
        package='titration_robot_cell_moveit_control',
        executable='move_state_server',
        output='screen',
    )
    
    return LaunchDescription([
        pipette_server_node,
        move_pose_server_node,
        move_state_server_node,
    ])