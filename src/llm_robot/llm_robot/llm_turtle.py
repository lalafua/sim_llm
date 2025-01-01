import rclpy, json, math
from rclpy.node import Node
from my_interfaces.srv import Command
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String


class TurtleNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Node {} has been created.".format(name))

        # Create publisher to control turtlebot
        self.turtle_control_publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Publisher 'cmd_vel' has been created.")
        self.vel_msg = Twist()

        # Create subscriber to get turtlebot pose
        self.turtle_pose_subscription_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.get_logger().info("Node has subscribed to '/turtle1/pose' ")
        self.current_pose = Pose()

        # Create subscriber to get camera message   
        self.camera_subscriber_ = self.create_subscription(String, "/camera/recognize", self.handle_camera, 10)
        self.get_logger().info("Node has subscribed to '/camera/recognize' ")  

        # Create server to receive command
        self.command_server_ = self.create_service(Command, 'nlp/nlp_cmd', self.handle_request)
        self.get_logger().info("Service 'nlp/nlp_cmd' has been created.")

    def pose_callback(self, msg):
        """
        get turtle pose

        Args:
            msg (Pose): the message from 'turtlebot' node
        
        Returns:
            None
        """

        self.current_pose = msg
        self.get_logger().info("turtle pose: {}".format(self.current_pose))
    
    def move_to_target(self, target_x, target_y):
        """
        control the turtlebot to move to target

        Args:
            target_x (float): target x
            target_y (float): target y
        """
        
        target_pose = Pose()
        target_pose.position.x = target_x
        target_pose.position.y = target_y

        distance_tolerance = 0.1

        while self.get_distance(self.current_pose, target_pose) >= distance_tolerance:
            self.vel_msg.linear.x = 1.5 * self.get_distance(self.current_pose, target_pose)
            self.vel_msg.angular.z = 4 * (math.atan2(target_pose.position.y - self.current_pose.position.y, target_pose.position.x - self.current_pose.position.x) - self.current_pose.orientation.z)

            self.turtle_control_publisher_.publish(self.vel_msg)
        self.get_logger().info(f'Moving to goal: ({target_x}, {target_y})')

        self.stop()
    
    def get_distance(self, pose1, pose2):
        """
        Calculate the distance between two positions
        """

        return math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2)   

    def stop(self):
        """
        stop the turtlebot
        """

        self.turtle_control_publisher_.publish(self.vel_msg)
        self.get_logger().info("Turtle has stopped.")

    def cruises(self):
        """
        control the turtlebot to cruises
        """
    
    def handle_camera(self, msg):
        """
        callback function to handle the message from 'camera' node

        Args:
            msg (String): the message from 'camera' node
        """

        self.get_logger().info("Received: {}".format(msg.data))

    def handle_request(self, request, response):
        """
        server callback function to handle the request from 'llm_nlp' node

        Args:
            request (Command.Request (string))
            response (Command.Response (bool))

        Returns:
            response (Command.Response): response to 'llm_nlp' node
        """

        try:
            self.parser_map(request.command)
            response.is_success = True
        except Exception as e:
            self.get_logger().error("Error: {}".format(e))
            response.is_success = False
        
        return response
    
    def parser_map(self, cmd):
        """
        parser the command from 'llm_nlp' node, then control the turtlebot

        Args:
            cmd (str): json string from nlp node

        Returns:
            None
        """

        commands = json.loads(cmd)['answer']['commands']
        
        for item in commands:
            command = item['command']
            parms = item['parms']
            if command in command_map:
                command_map[command](parms)

        command_map = {
            "find": lambda parms : self.find(parms['goal'])
        }
        
    def find(self, goal):
        """
        find the goal

        Args:
            goal (str): the goal to find
        """


        


def main(args=None):
    rclpy.init(args=args)
    turtle_node = TurtleNode("llm_turtle")
    rclpy.spin(turtle_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()