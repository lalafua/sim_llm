import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class cameraNode(Node):
    def __init__(self, name):
        super.__init__(name)
        self.get_logger().info("Node {} has been created.".format(name))    
        self.camera_publisher_ = self.create_publisher(msg_type=String, topic="/camera/recognize", qos_profile=10)
        self.timer_ = self.create_timer(10.0, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = "bottle"
        self.camera_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    camera_node = cameraNode("camera_node")
    rclpy.spin(camera_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()