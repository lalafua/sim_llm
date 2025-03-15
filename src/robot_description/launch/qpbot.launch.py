import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

RVIZ2_ENABLE = False

def generate_launch_description():

    package_name = 'robot_description'

    ld = LaunchDescription()

    package_share = get_package_share_directory(package_name=package_name)

    # robot file path
    xacro_file_path = os.path.join(package_share, 'urdf', 'qpbot.xacro')
    urdf_file_path = os.path.join(package_share, 'urdf', 'qpbot.urdf')

    # convert xacro to urdf
    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file_path, '-o',urdf_file_path],
        output='screen'
    )

    # declare arguments
    gazebo_declare_robot = DeclareLaunchArgument(
        name='robot',
        default_value=urdf_file_path,
        description='Robot file to load for gazebo'
    )

    gazebo_declare_robot_x = DeclareLaunchArgument(
        name='robot_x',
        default_value='5.0',
        description='Robot x position'
    )

    gazebo_declare_robot_y = DeclareLaunchArgument(
        name='robot_y',
        default_value='1.0',
        description='Robot x position'
    )

    gazebo_declare_robot_z = DeclareLaunchArgument(
        name='robot_z',
        default_value='0.0',
        description='Robot x position'
    )

    gazebo_declare_robot_yaw = DeclareLaunchArgument(
        name='robot_yaw',
        default_value='-3.1',
        description='Robot yaw'
    )

    # spawn a robot in gazebo world
    spawn_robot = Node(
        name='spawn_robot',
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen', 
        arguments=[
            '-entity', 'qpbot',
            '-file', LaunchConfiguration('robot'),
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z'),
            '-Y', LaunchConfiguration('robot_yaw'),
        ]
    )

    # publish robot state (TF) to the topic robot_description
    state_publisher = Node(
        name='state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=[urdf_file_path]
    )

    # Rviz2 node
    rviz2_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(package_share, 'rviz2', 'qpbot.rviz')
        ]
    )

    ld.add_action(xacro_to_urdf)
    ld.add_action(gazebo_declare_robot)
    ld.add_action(gazebo_declare_robot_x)
    ld.add_action(gazebo_declare_robot_y)
    ld.add_action(gazebo_declare_robot_z)
    ld.add_action(gazebo_declare_robot_yaw)
    ld.add_action(spawn_robot)
    ld.add_action(state_publisher)
    
    if RVIZ2_ENABLE:
        ld.add_action(rviz2_node)

    return ld