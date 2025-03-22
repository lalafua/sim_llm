from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult    
import rclpy
from rclpy.clock import Clock
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException  

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

    tf2_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf2_buffer, navigator)

    def get_pose(target_frame='base_footprint', source_frame='map'):    
        try:
            transform = tf2_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,  
                time=rclpy.time.Time(),
                timeout=rclpy.time.Duration(seconds=1.0)    
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            return {'x': f'{x:.3f}', 'y': f'{y:.3f}', 'z': f'{z:.3f}'} 

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            navigator.get_logger().error(str(e))
            return None

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

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i%5 == 0:
            navigator.get_logger().info(
                'Executing current waypoint:\n'
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_poses))
                + '\n'
                + str(get_pose())
            )
    
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