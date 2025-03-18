import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_name = 'llm_robot'

def generate_launch_description():
    ld = LaunchDescription()

    llm_robot_dir = get_package_share_directory(package_name='llm_robot')
    robot_description_dir = get_package_share_directory(package_name='robot_description')
    robot_navigation2_dir = get_package_share_directory(package_name='robot_navigation2') 

    test_node = Node(
        name='test',
        package='llm_robot',
        executable='test',
    )

    robot_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_description_dir, 'launch', 'robot_spawn.launch.py'))
    )

    robot_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_navigation2_dir, 'launch', 'qpbot_nav2.launch.py'))
    )

    ld.add_action(test_node)
    ld.add_action(robot_spawn_launch)
    ld.add_action(robot_nav2_launch)

    return ld