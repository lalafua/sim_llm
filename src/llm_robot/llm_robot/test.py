from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
from copy import deepcopy


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = navigator.get_clock().now().to_msg()
    init_pose.pose.position.x = 5.0  
    init_pose.pose.position.y = 1.0  
    init_pose.pose.orientation.z = 0.0
    init_pose.pose.orientation.w = 1.0

    navigator.setInitialPose(init_pose)

    goal_pose = deepcopy(init_pose)
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -4.0  
    goal_pose.pose.position.y = -3.0  
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    navigator.goToPose(goal_pose)

    while not navigator.isNavComplete():
        feedback = navigator.getFeedback()
        if feedback and feedback.navigation_state != 'IDLE':
            print(f"当前导航状态: {feedback.navigation_state}")

    result = navigator.getResult()
    if result == BasicNavigator.RESULT_SUCCEEDED:
        print('到达目标点!')
    elif result == BasicNavigator.RESULT_CANCELED:
        print('导航被取消!')
    elif result == BasicNavigator.RESULT_FAILED:
        print('导航失败!')

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()