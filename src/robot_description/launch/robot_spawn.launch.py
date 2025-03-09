import os 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'robot_description'

    ld = LaunchDescription()

    # package share direactory path
    robot_description_share = get_package_share_directory(package_name=package_name)

    # world file path
    world_file_path = os.path.join(robot_description_share, 'worlds', 'ROS-Academy.world')

    # gazebo model setup
    gazebo_model_path = os.path.join(robot_description_share, 'models')
    gazebo_model_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=[gazebo_model_path, ":", "$GAZEBO_MODEL_PATH"]    
        )
    
    # gazebo auguments declare
    # world file declare
    gazebo_declare_world = DeclareLaunchArgument(
        name='world',
        default_value=world_file_path,
        description='World file to load for gazebo'
    )

    # # simulator declare
    # gazebo_declare_simulator = DeclareLaunchArgument(
    # name='headless',
    # default_value='False',
    # description='Whether to execute gzclient'
    # )
     
    # # use_sim_time declare
    # gazebo_declare_use_sim_time = DeclareLaunchArgument(
    #     name='use_sim_time',
    #     default_value='true',
    #     description='Use simulation (Gazebo) clock if true'
    # )
    
    # # use simulator declare 
    # gazebo_declare_use_simulator = DeclareLaunchArgument(
    #     name='use_simulator',
    #     default_value='True',
    #     description='Whether to start the simulator'
    #     )

    gazebo_server = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world')],
        output='screen'
    )

    gazebo_entity_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_share, 'launch', 'xbot-u.launch.py')
        )
    )
    
    ld.add_action(gazebo_model_env)
    ld.add_action(gazebo_declare_world)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_entity_spawn)

    return ld