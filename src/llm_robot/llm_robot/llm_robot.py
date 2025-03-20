import rclpy, json, threading    
from rclpy.node import Node
from rclpy.duration import Duration 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion  
from my_interfaces.srv import Command
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator

class llmRobotNode(Node):
    def __init__(self, name):       
        super().__init__(name)
        self.get_logger().info("Node {} has been created.".format(name))

        self.callback_group_ = MutuallyExclusiveCallbackGroup()

        self.lock = threading.Lock()

        self.camera_init()  
        self.navigator_init()   
        self.parser_init()

    def camera_init(self):
        """
        Initialize the camera subscriber
        """

        self.camera_subscriber_ = self.create_subscription(
            msg_type=String,
            topic="/camera/recognized",
            callback=self.camera_callback,
            qos_profile=10,
            callback_group=self.callback_group_
        )    
        self.get_logger().info("Node has subscribed to '/camera/recognized' ")  


    def camera_callback(self, msg):
        """
        callback function to handle the message from 'camera' node

        Args:
            msg (String): the message from 'camera' node
        """

        with self.lock:
            self.recognized_goal = msg.data
            self.get_logger().info("Recognized goal: {}".format(self.recognized_goal))
    
    def get_recognized_goal(self):
        """
        get the recognized goal

        Returns:
            str: the recognized goal
        """

        return self.recognized_goal

    def navigator_init(self):
        """
        Initialize the navigator
        """

        self.navigator = BasicNavigator()
        # Wait until nav2 is active 
        self.navigator.waitUntilNav2Active()

        self.navigator.setInitialPose(
            self.create_pose(
                frame_id='map',
                x=0.0,
                y=0.0,
                z=0.0   
            )
        )

        # Get real-time acml pose
        self.pose_subscriber_ = self.create_subscription(
            msg_type=PoseStamped,
            topic="/amcl_pose",
            callback=self.pose_callback,
            qos_profile=10,
        )

    def pose_callback(self, msg):
        """
        callback function to handle the message from 'amcl_pose' topic

        Args:
            msg (PoseStamped): the message from 'amcl_pose' topic
        """

        with self.lock:
            self.current_pose = msg
            self.get_logger().info("Current pose: {}".format(self.current_pose))
    
    def get_current_pose(self):
        """
        get the current pose

        Returns:
            PoseStamped: the current pose
        """

        return self.current_pose


    def create_pose(self, frame_id, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
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
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose_msg
        return pose_stamped
    
    def cancel_navigation(self):
        """
        cancel the navigation
        """

        self.navigator.cancelTask()

    def parser_init(self):
        """
        Initialize the parser commander
        """

        self.parser_success = False
        self.parser_thread = None
        self.parser_event = threading.Event()

        # Create server to receive command
        self.command_server_ = self.create_service(
            srv_type=Command,
            srv_name='/llm_nlp/cmd',
            callback=self.handle_request
        )
        self.get_logger().info("Service '/llm_nlp/cmd' has been created.")

    def handle_request(self, request, response):
        """
        server callback function to handle the request from 'llm_nlp' node

        Args:
            request (Command.Request (string))
            response (Command.Response (bool))

        Returns:
            response (Command.Response): response to 'llm_nlp' node
        """

        if not request.command:
            self.get_logger().error("Received empty command.")
            response.is_success = False  
            return  response

        try:
            self.parser_map(request.command)
            self.parser_event.wait()
            response.is_success = self.parser_success 
        except KeyError as e:
            self.get_logger().error("Key error: {}".format(e))
            response.is_success = False 
        except json.JSONDecodeError as e:   
            self.get_logger().error("Json decode error: {}".format(e))
            response.is_success = False 
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

        commands = json.loads(cmd)["commands"]
        print(commands) 
        
        command_map = {
            "find": lambda parms : self.find(parms["goal"])
        }

        for item in commands:
            command = item["command"]
            parms = item["parms"]
            if command in command_map:
                command_map[command](parms)
            else:
                self.get_logger().error("Command {} not found.".format(command))

    def find(self, goal):
        """
        find the goal

        Args:
            goal (str): the goal to find
        """
   
        self.parser_success = False 
        self.parser_event.clear()

        # Define fixed patrol poses
        patrol_poses = [
            (-4.0, -4.0, 0.0),
            (4.0, -4.0, 0.0),
            (4.0, 4.0, 0.0),
            (0.0, 0.0, 0.0),
        ]

        waypoints_pose = []

        for pose in patrol_poses:
            waypoints_pose.append(
                self.create_pose(
                    frame_id='map',
                    x=pose[0],
                    y=pose[1],
                    z=pose[2],
                )
            )
    
        # Send waypoints pose to navigator
        def send_waypoints():
            self.nav_start = self.get_clock().now()
            self.navigator.followWaypoints(waypoints_pose)

        def is_goal_detected():
            return self.get_recognized_goal() == goal
        
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i%5 == 0:
                self.get_logger().info(
                    'Executing current waypoint:\n'
                    + str(feedback.current_waypoint + 1)
                    + '/'
                    + str(len(waypoints_pose))
                    + '\n'
                    + str(self.get_current_pose())
                )
            
                now = self.get_clock().now()

                if now - self.nav_start > Duration(seconds=600.0):
                    self.get_logger().error("Navigation timeout.")
                    self.cancel_navigation()

            if is_goal_detected():
                self.get_logger().info("Goal detected.")
                self.parser_success = True
                self.cancel_navigation()   
                
                    
                
        
        result = self.navigator.getResult()
        if result == self.TaskResult.SUCCESS:
            self.get_logger("Task completed successfully.")
        elif result == self.TaskResult.CANCELED:
            self.get_logger("Task canceled.") 
        elif result == self.TaskResult.FAILURE:
            self.get_logger("Task failed.")   
        else:
            self.get_logger("Unknown result.")    
            

def main(args=None):
    rclpy.init(args=args)

    node = llmRobotNode("llm_robot")
    node.update_joint_speed([15.0, -15.0])
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()