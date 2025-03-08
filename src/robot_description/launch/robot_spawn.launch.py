import os 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'robot_description'

    ld = LaunchDescription()

    # package share direactory path
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    robot_description_share = get_package_share_directory(package_name=package_name)

    # world file path
    world_file_path = os.path.join(robot_description_share, 'worlds', 'ROS-Academy.world')

    # gazebo model setup
    gazebo_model_path = os.path.join(robot_description_share, 'models')
    gazebo_model_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=[gazebo_model_path, ":", "$GAZEBO_MODEL_PATH"]    
        )
    
    # Declare the launch arguments  
    world_declare = DeclareLaunchArgument(
        name='world',
        default_value=world_file_path,
        description='World file to load for gazebo'
    )

    gazebo = Node(
        package='gazebo_ros',
        executable='gazebo',
        output='screen',
        arguments=[
            '--verbose',
            '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so',
            LaunchConfiguration('world')
            ] 
    )

    ld.add_action(gazebo_model_env)
    ld.add_action(world_declare)
    ld.add_action(gazebo)

    return ld