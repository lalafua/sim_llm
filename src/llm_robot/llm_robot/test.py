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
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.z = 0.0
    init_pose.pose.orientation.w = 1.0

    navigator.setInitialPose(init_pose)

    partrol_pose = [
        (-4.0, -3.0, 0.0),
        (4.2, 7.4, 0.0),
        (5.0, 1.0, 0.0)
    ]

    goal_pose = deepcopy(init_pose)

    for pose in partrol_pose:
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]

        # 关键修改：将 navigator.goToPose() 移动到 for 循环内部!**
        navigator.goToPose(goal_pose)  # 每次循环都发送新的导航目标

        while not navigator.isTaskComplete():
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