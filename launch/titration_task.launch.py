from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    tree_path = PathJoinSubstitution ( [get_package_share_directory('titration_robot_cell_moveit_control'), 'tree', 'titration_task.xml'])
    
    bt_node = Node(
        package='titration_robot_cell_moveit_control',
        executable='titration_task',
        output='screen',
        parameters=[{'tree_xml_file': tree_path}],
    )
    
    return LaunchDescription([bt_node])