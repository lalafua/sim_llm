from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult    
import rclpy
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class PoseStampedFactory:
    @classmethod
    def create_pose(cls, frame_id, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        Create PoseStamped message.

        Args:
            frame_id (str): frame id.
            x, y, z (float): coordinates.
            qx, qy, qz, qw (float): quaternion.

        Returns:
            PoseStamped: PoseStamped message.
        """
        pose_msg = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)
        )

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = Clock().now().to_msg()
        pose_stamped.pose = pose_msg
        return pose_stamped

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    init_pose = PoseStampedFactory.create_pose(
        frame_id='map',
        x=0.0,
        y=0.0,
        z=0.0,
        )

    navigator.setInitialPose(init_pose)

    patrol_poses = [
        (-4.0, -4.0, 0.0),
        (4.0, -4.0, 0.0),
        (4.0, 4.0, 0.0),
        (0.0, 0.0, 0.0),
    ]

    goal_poses = []

    for pose in patrol_poses:
        goal_poses.append(
            PoseStampedFactory.create_pose(
                frame_id='map',
                x=pose[0],
                y=pose[1],
                z=pose[2],
            )
        )

    navigator.followWaypoints(goal_poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback.navigation_duration > 600:
            navigator.cancelTask()
    
    result = navigator.getResult()
    if result == TaskResult.SUCCESS:
        print("Task completed successfully.")
    elif result == TaskResult.CANCELED:
        print("Task canceled.") 
    elif result == TaskResult.FAILURE:
        print("Task failed.")   
    else:
        print("Unknown result.")    

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()