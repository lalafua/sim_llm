import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument    
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


package_name = 'robot_description'
package_share = get_package_share_directory(package_name=package_name)

def declare_arguments():
    use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    resolution = DeclareLaunchArgument(
        name='resolution',
        default_value='0.05',
        description='Occupancy grid resolution'
    )

    publish_period_sec = DeclareLaunchArgument(
        name='publish_period_sec',
        default_value='1.0',
        description='Occupancy grid publication period'
    )

    configuration_directory = DeclareLaunchArgument(
        name='configuration_directory',
        default_value=os.path.join(
            package_share, 'config'),
        description='Full path to config directory'
    )

    configuration_basename = DeclareLaunchArgument(
        name='configuration_basename',
        default_value='qpbot_slam_2d.lua',
        description='Basename of configuration file'
    )

    rviz2_directory = DeclareLaunchArgument(
        name='rviz2_directory',
        default_value=os.path.join(
            package_share, 'rviz2'),
        description='Full path to rviz2 directory' 
    )

    return [use_sim_time, resolution, publish_period_sec, configuration_directory, configuration_basename, rviz2_directory] 

def generate_launch_description():
    ld = LaunchDescription()

    delare_args = declare_arguments()
    [ld.add_action(arg) for arg in delare_args]

    cartographer_node = Node(
        name='cartographer_node',
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
        arguments=[
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basename', LaunchConfiguration('configuration_basename'),  
        ],
    )

    cartographer_occupancy_grid_node = Node(
        name='cartographer_occupancy_grid_node',
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[{
            'use_sim_time':LaunchConfiguration('use_sim_time'),
        }],
        arguments=[
            '-resolution', LaunchConfiguration('resolution'),
            '-publish_period_sec', LaunchConfiguration('publish_period_sec'),
        ],
    )

    rviz2_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(package_share, 'rviz2', 'cartographer.rviz')],
    )

    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz2_node)

    return ld