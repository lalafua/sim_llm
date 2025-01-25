import os
from launch import LaunchDescription    
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    package_name = "llm_robot"
    car_model_name = "car_base.urdf"
    rviz_config_name = "llm_robot.rviz"
    
    ld = LaunchDescription()

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    camera_node = Node(
        package="llm_robot",
        executable="camera",
        name="camera",
    )

    llm_robot_node = Node(
        package="llm_robot",
        executable="llm_robot",
        name="llm_robot",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        arguments=[os.path.join(pkg_share, "urdf/{}".format(car_model_name))],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        #arguments=["-d", os.path.join(pkg_share, "rviz/{}".format(rviz_config_name))],
    )

    #ld.add_action(camera_node)
    ld.add_action(llm_robot_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
     
    return ld







