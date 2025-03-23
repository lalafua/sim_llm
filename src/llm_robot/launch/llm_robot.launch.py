import os
from launch import LaunchDescription    
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

package_name = 'llm_robot'

def generate_launch_description():
    ld = LaunchDescription()

    llm_robot_dir = get_package_share_directory(package_name='llm_robot')
    robot_description_dir = get_package_share_directory(package_name='robot_description')
    robot_navigation2_dir = get_package_share_directory(package_name='robot_navigation2') 


    # Launch llm_robot to control the robot 
    llm_robot_node = Node(
        name='llm_robot',   
        package='llm_robot',
        executable='llm_robot',
    )

    # Launch camera node to recognize the goal
    camera_node = Node(
        name='camera',
        package='llm_robot',
        executable='camera',    
    )

    # Launch llm_nlp node to process neutral language
    llm_nlp_node = Node(
        name='llm_nlp',
        package='llm_robot',
        executable='llm_nlp',
    )

    # Launch the robot spawn to spawn the robot in the gazebo world 
    robot_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_description_dir, 'launch', 'robot_spawn.launch.py'))
    )

    # Launch the robot navigation2 to start navigation2 map server and planner server
    robot_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_navigation2_dir, 'launch', 'qpbot_nav2.launch.py'))
    )

    ld.add_action(llm_robot_node)
    ld.add_action(camera_node)
    #ld.add_action(llm_nlp_node) 
    ld.add_action(robot_spawn_launch)
    ld.add_action(robot_nav2_launch)

    return ld







