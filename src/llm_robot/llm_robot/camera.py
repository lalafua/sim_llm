import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class cameraNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Node {} has been created.".format(name))    
        self.camera_publisher_ = self.create_publisher(msg_type=String, topic="/camera/recognized", qos_profile=10)
        self.timer_ = self.create_timer(20, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = "bottle"
        self.camera_publisher_.publish(msg)
        self.get_logger().info("Published to topic '/camera/recognized': '{}'".format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    camera_node = cameraNode("camera_node")
    executor = MultiThreadedExecutor()
    executor.add_node(camera_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass        
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()