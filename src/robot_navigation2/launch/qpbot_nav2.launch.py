import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_name = 'robot_navigation2'

def generate_launch_description():
    ld = LaunchDescription()
    
    robot_navigation2_dir = get_package_share_directory(package_name='robot_navigation2')
    nav2_bringup_dir = get_package_share_directory(package_name='nav2_bringup')
    robot_description_dir = get_package_share_directory(package_name='robot_description')

    map_dir = os.path.join(robot_description_dir, 'map')
    param_dir = os.path.join(robot_navigation2_dir, 'param')
    rviz2_dir = os.path.join(robot_navigation2_dir, 'rviz2')

    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    param_file = LaunchConfiguration('param_file')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_a = LaunchConfiguration('robot_a')

    declare_map_file_cmd = DeclareLaunchArgument(
        name='map_file',
        default_value=os.path.join(map_dir, 'ISCAS_Museum.yaml'),
        description='Declare map file path'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_param_file_cmd = DeclareLaunchArgument(
        name="param_file",
        default_value=os.path.join(param_dir, 'qpbot_nav2.yaml')
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map':map_file,
            'use_sim_time':use_sim_time,
            'params_file':param_file
        }.items()
    )

    amcl_node = Node(
        name='amcl',
        package='nav2_amcl',
        executable='amcl',
        parameters=[{
            'initial_pose/x':robot_x,
            'initial_pose/y':robot_y,
            'initial_pose/a':robot_a
        }]
    )

    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_param_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(nav2_bringup_launch)
    ld.add_action(amcl_node)

    return ld
