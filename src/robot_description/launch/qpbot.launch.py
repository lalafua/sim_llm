import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

ENABLE_RVIZ2 = False
package_name = 'robot_description'

def generate_launch_description():
    ld = LaunchDescription()

    robot_description_dir = get_package_share_directory(package_name='robot_description')

    # sub file path
    urdf_dir = os.path.join(robot_description_dir, 'urdf')
    rviz2_dir = os.path.join(robot_description_dir, 'rviz2')


    xacro_file_path = os.path.join(urdf_dir, 'qpbot.xacro')
    urdf_file_path = os.path.join(urdf_dir, 'qpbot.urdf')

    # convert xacro to urdf
    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file_path, '-o',urdf_file_path],
        output='screen'
    )

    robot_file = LaunchConfiguration('robot_file')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_a = LaunchConfiguration('robot_a')    
    
    # declare arguments
    declare_robot_file_cmd =  DeclareLaunchArgument(
        name='robot_file',
        default_value=os.path.join(urdf_dir, 'qpbot.urdf'),
        description='Robot file to load for gazebo'
    )

    declare_robot_x_cmd = DeclareLaunchArgument(
        name='robot_x',
        default_value='5.0',
        description='Robot x position'
    )

    declare_robot_y_cmd = DeclareLaunchArgument(
        name='robot_y',
        default_value='1.0',
        description='Robot x position'
    )

    declare_robot_z_cmd = DeclareLaunchArgument(
        name='robot_z',
        default_value='0.0',
        description='Robot x position'
    )

    declare_robot_a_cmd = DeclareLaunchArgument(
        name='robot_a',
        default_value='0.0',
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
            '-file', robot_file,
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
            '-d', os.path.join(rviz2_dir, 'qpbot.rviz')
        ]
    )

    ld.add_action(xacro_to_urdf)
    ld.add_action(declare_robot_file_cmd)
    ld.add_action(declare_robot_x_cmd)
    ld.add_action(declare_robot_y_cmd)
    ld.add_action(declare_robot_z_cmd)
    ld.add_action(declare_robot_a_cmd)
    ld.add_action(spawn_robot)
    ld.add_action(state_publisher)
    
    if ENABLE_RVIZ2:
        ld.add_action(rviz2_node)

    return ld