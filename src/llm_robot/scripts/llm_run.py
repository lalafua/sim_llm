#!/usr/bin/env python

import rospy, json, math, time, threading
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from llm_robot.srv import trans, transResponse

class llmRobotNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        rospy.loginfo("Node {} has been created.".format(name))

        # Create subscriber to get turtle pose
        self.turtle_pose_sub = rospy.Subscriber(
            name="/turtle1/pose",
            data_class=Pose,
            callback=self.pose_callback,
            queue_size=10
        )
        rospy.loginfo("Node has subscriped to '/turtle1/pose'")

        # Create subscriber to get camera message
        self.camera_sub = rospy.Subscriber(
            name="/camera/recognized",
            data_class=String,
            callback=self.camera_callback,
            queue_size=10
        )
        rospy.loginfo("Node has subscriped to '/camera/recognized'")
        self.recognized_object = "" # Initialize recognized object

        # Create publisher to control turtlesim
        self.turtle_control_pub = rospy.Publisher(
            name="/turtle1/cmd_vel",
            data_class=Twist,
            queue_size=10
        )
        rospy.loginfo("Node has created publisher to '/turtle1/cmd_vel'")
        self.vel_msg = Twist() # Initialize velocity message
        self.current_pose = Pose() # Initialize current pose
        self.lock = threading.Lock() # Create lock for current_pose

        # Create service for llm_nlp node
        self.service_name = "/trans"
        self.trans_service = rospy.Service(
            name=self.service_name,
            service_class=trans,
            handler=self.handle_request
        )
        self.parser_success = False
        self.parser_event = threading.Event()

    def pose_callback(self, msg):
        """
        get turtle pose

        Args:
            msg (Pose): the message from '/turtle1/pose'
        
        Returns:
            None
        """

        with self.lock:
            self.current_pose = msg
        
    def camera_callback(self, msg):
        """
        callback function to handle the message from '/camera/recognized'

        Args:
            msg (String): the message from '/camera/recoginzed'
        """

        with self.lock:
            self.recognized_object = msg.data
            rospy.loginfo("Recognized object: {}".format(self.recognized_object))

    def move_to_target(self, target_x, target_y, object=""):
        """
        control the turtlebot to move to target, if object is find, stop

        Args:
            target_x (float): target x
            target_y (float): target y
            goal (str): the object to find
        """

        target_pose = Pose()
        target_pose.x = target_x
        target_pose.y = target_y
        rospy.loginfo("Turtle is moving to target: \n{}".format(target_pose))

        while not rospy.is_shutdown():
            with self.lock:
                cur_pose = self.current_pose
            distance = math.hypot(target_pose.x - cur_pose.x, target_pose.y - cur_pose.y)
            if distance < 0.1:
                break
            angle = math.atan2(target_pose.y - cur_pose.y, target_pose.x - cur_pose.x)
            angular_diff = angle - cur_pose.theta
            angular_diff = (angular_diff + math.pi) % (2 * math.pi) - math.pi

            if object and object == self.recognized_object:
                self.parser_success = True
                self.parser_event.set()
                rospy.loginfo("Object found at position: \n{}".format(cur_pose)) 
                return
            
            if abs(angular_diff) > 0.1:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = angular_diff
                self.turtle_control_pub.publish(self.vel_msg)
            else:
                self.vel_msg.linear.x = min(1.5, distance)
                self.vel_msg.angular.z = 0.0    
                self.turtle_control_pub.publish(self.vel_msg)
            time.sleep(0.1)
        
        self.stop()
        rospy.loginfo("Turtle has reached \n{}".format(cur_pose))

    def stop(self):
        """
        stop the turtlebot
        """

        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.turtle_control_pub.publish(self.vel_msg)
        rospy.loginfo("Turtle has stopped.")

    def handle_request(self, request):
        """
        server callback function to handle the request from '/trans'

        Args:
            request (trans.Request (string))
            response (trans.Response (bool))

        Returns:
            response (trans.Response): response to '/trans'
        """

        if not request.command:
            rospy.logerr("Empty command received.") 
            return transResponse(is_success=False)
        
        try:
            self.parser_map(request.command)
            self.parser_event.wait()
            return transResponse(is_success=self.parser_success)
        except KeyError as e:
            rospy.logerr("KeyError: {}".format(e))
            return transResponse(is_success=False)
        except json.JSONDecodeError as e:
            rospy.logerr("Json decode error: {}".format(e))
            return transResponse(is_success=False)
        except Exception as e:
            rospy.logerr("Error: {}".format(e))
            return transResponse(is_success=False)
        
    def parser_map(self, cmd):
        """
        parser the command from '/trans', then control the turtlebot

        Args:
            cmd (str): json string from nlp node

        Returns:
            None
        """

        commands = json.loads(cmd)["commands"]
        rospy.loginfo("Parserd commands: {}".format(commands))

        command_map = {
            "find": lambda parms: self.find(parms["object"])
        }

        for item in commands:
            command = item["command"]
            parms = item["parms"]
            if command in command_map:
                result = command_map[command](parms)
                if isinstance(result, threading.Thread):
                    result.join()
            else:
                rospy.logwarn("Command not found: {}".format(command))
                break
    
    def find(self, object):
        """
        find the object

        Args:
            object (str): the object to find

        Returns:
            None
        """

        rospy.loginfo("Start finding :{}".format(object))
        self.parser_success = False 
        self.parser_event.clear()
        
        patrol_points = [
            (1.0, 1.0),
            (9.0, 1.0),
            (9.0, 9.0),
            (1.0, 9.0)
        ]

        with self.lock:
            cur_pose = self.current_pose

        original_position = (cur_pose.x, cur_pose.y)
        rospy.loginfo("Original position: {}".format(original_position))

        def patrol():
            while not self.parser_success and not rospy.is_shutdown():
                for point in patrol_points:
                    if self.parser_success:
                        break
                    self.move_to_target(point[0], point[1], object)
                if not self.parser_success:
                    rospy.loginfo("Goal not found, continuing patrol.")
            self.move_to_target(original_position[0], original_position[1])
            rospy.loginfo("Returned to original position.")
        
        patrol_thread = threading.Thread(target=patrol)
        patrol_thread.start()

        return patrol_thread

def main():
    node = llmRobotNode(name="llm_robot")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    
if __name__ == "__main__":  
    main()