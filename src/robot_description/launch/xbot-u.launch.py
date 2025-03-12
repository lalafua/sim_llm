import os 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    package_name = 'robot_description'

    ld = LaunchDescription()

    robot_description_share = get_package_share_directory(package_name=package_name)

    # robot file path
    robot_file_path = os.path.join(robot_description_share, 'urdf', 'robot.xacro')

    robot_description_obj = Command(command=['xacro', ' ', robot_file_path])

    # declare arguments
    gazebo_declare_robot = DeclareLaunchArgument(
        name='robot',
        default_value=robot_file_path,
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
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        namespace='xbot',
        arguments=[
            '-entity', 'xbot',
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z'),
            '-Y', LaunchConfiguration('robot_yaw'),
            '-topic', 'robot_description',
        ]
    )

    # publish robot state (TF) to the topic robot_description_obj
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='xbot',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_obj, value_type=str),
            'publish_frequency': 20.0,
        }]
    )

    ld.add_action(gazebo_declare_robot)
    ld.add_action(gazebo_declare_robot_x)
    ld.add_action(gazebo_declare_robot_y)
    ld.add_action(gazebo_declare_robot_z)
    ld.add_action(gazebo_declare_robot_yaw)
    ld.add_action(spawn_robot)
    ld.add_action(state_publisher)

    return ld