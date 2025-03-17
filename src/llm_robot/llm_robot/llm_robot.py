import rclpy, json, math, time,threading    
from rclpy.node import Node
from my_interfaces.srv import Command
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator
from copy import deepcopy

class llmRobotNode(Node):
    def __init__(self, name):       
        super().__init__(name)
        self.get_logger().info("Node {} has been created.".format(name))

        self.callback_group_ = MutuallyExclusiveCallbackGroup()

        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )
        self.init_joint_state()
        self.pub_rate = self.create_rate(30)
        threading.Thread(target=self.thread_joint_state).start()

        # # Create subscriber to get turtle pose
        # self.turtle_pose_subscription_ = self.create_subscription(
        #     Pose,
        #     '/turtle1/pose',
        #     self.pose_callback,
        #     10,
        #     callback_group=self.callback_group_)    
        # self.get_logger().info("Node has subscribed to '/turtle1/pose' ")

        # Create subscriber to get camera message   
        self.camera_subscriber_ = self.create_subscription(
            String,
            "/camera/recognized",
            self.camera_callback,
            10,
            callback_group=self.callback_group_)    
        self.get_logger().info("Node has subscribed to '/camera/recognized' ")  

        self.lock = threading.Lock()

        # # Create publisher to control turtle
        # self.turtle_control_publisher_ = self.create_publisher(
        #     Twist,
        #     '/turtle1/cmd_vel',
        #     10)
        # self.get_logger().info("Publisher 'cmd_vel' has been created.")
        # self.vel_msg = Twist()
        # self.current_pose = Pose()
        # self.recognized_goal = ""

        self.parser_success = False
        self.parser_thread = None
        self.parser_event = threading.Event()

        # Create server to receive command
        self.command_server_ = self.create_service(
            Command,
            '/llm_nlp/cmd',
            self.handle_request)
        self.get_logger().info("Service '/llm_nlp/cmd' has been created.")


    def init_joint_state(self):
        """
        initialize the joint state
        """

        self.joint_speed = [0.0, 0.0]
        self.joint_state = JointState()
        self.joint_state.header.stamp = self.get_clock().now().to_msg() 
        self.joint_state.header.frame_id = "car_frame"
        self.joint_state.name = ["left_wheel_joint", "right_wheel_joint"]
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = self.joint_speed
        self.joint_state.effort = []

    def update_joint_speed(self, speed):
        """
        update the joint speed

        Args:
            speed (list): the speed of left and right wheels
        """

        self.joint_speed = speed

    def thread_joint_state(self):
        """
        thread for controlling the joint state
        """

        last_time  = time.time()
        while rclpy.ok():
            delta_time =  time.time()-last_time
            last_time = time.time()
            
            # Update joint state position
            self.joint_state.position[0]  += delta_time*self.joint_state.velocity[0]
            self.joint_state.position[1]  += delta_time*self.joint_state.velocity[1]
            
            # Update joint state velocity   
            self.joint_state.velocity = self.joint_speed
            
            # Update header
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            
            # Publish joint state
            self.joint_state_publisher_.publish(self.joint_state)
            self.pub_rate.sleep()
          
    def camera_callback(self, msg):
        """
        callback function to handle the message from 'camera' node

        Args:
            msg (String): the message from 'camera' node
        """

        with self.lock:
            self.recognized_goal = msg.data
            self.get_logger().info("Recognized goal: {}".format(self.recognized_goal))

    
    def navigator_init(self):
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        

    def move_to_target(self, target_x, target_y, goal=""):
        """
        control the turtlebot to move to target

        Args:
            target_x (float): target x
            target_y (float): target y
            goal (str): the goal to find
        """
        
        target_pose = Pose()
        target_pose.x = target_x
        target_pose.y = target_y

        while True:
            if self.current_pose is not None:   
                distance_x = target_pose.x - self.current_pose.x
                distance_y = target_pose.y - self.current_pose.y
                distance = math.sqrt(distance_x**2 + distance_y**2)
                
                if distance < 0.1:
                    break
                
                angle = math.atan2(distance_y, distance_x)
                angular_diff = angle - self.current_pose.theta  

            # Rotate to the target direction
            while abs(angular_diff) > 0.01:
                if goal == self.recognized_goal:
                    self.parser_success = True
                    self.get_logger().info("Goal found at position: {}".format(self.current_pose))
                    self.parser_event.set()
                    return
                angular_diff = angle - self.current_pose.theta
                angular_diff = (angular_diff + math.pi) % (2 * math.pi) - math.pi
                
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = angular_diff
                self.turtle_control_publisher_.publish(self.vel_msg)
                time.sleep(0.1)

            # Move straight to the target
            while distance > 0.1:
                if goal == self.recognized_goal:
                    self.parser_success = True
                    self.get_logger().info("Goal found at position: {}".format(self.current_pose))
                    self.parser_event.set()
                    return     
                distance_x = target_pose.x - self.current_pose.x
                distance_y = target_pose.y - self.current_pose.y
                distance = math.sqrt(distance_x**2 + distance_y**2)
                
                self.vel_msg.linear.x = min(1.5, distance)
                self.vel_msg.angular.z = 0.0
                self.turtle_control_publisher_.publish(self.vel_msg)
                time.sleep(0.1)
        
        self.stop()
        self.get_logger().info("Turtle has reached {}".format(self.current_pose))
        

    def stop(self):
        """
        stop the turtlebot
        """

        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.turtle_control_publisher_.publish(self.vel_msg)
        self.get_logger().info("Turtle has stopped.")
    

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

        # Define fixed patrol points
        patrol_points = [
            (1.0, 1.0),
            (9.0, 1.0),
            (9.0, 9.0),
            (1.0, 9.0)
        ]
            
        # Store the original position
        original_position = (self.current_pose.x, self.current_pose.y)
        self.get_logger().info("Original position: {}".format(original_position))   

        def patrol():
            while not self.parser_success:
                for point in patrol_points:
                    if self.parser_success:
                        break
                    self.move_to_target(point[0], point[1], goal)

                if not self.parser_success:
                    self.get_logger().info("Goal not found, continuing patrol.")

            # If goal is found, return to the original position
            self.move_to_target(original_position[0], original_position[1])
            self.get_logger().info("Returned to original position.")

        # Start patrol in a new thread
        self.patrol_thread = threading.Thread(target=patrol)
        self.patrol_thread.start()

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