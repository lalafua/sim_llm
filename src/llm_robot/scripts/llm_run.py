#!/usr/bin/env python

import rospy, json, time, threading
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from llm_robot.srv import trans, transResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from actionlib.simple_action_client import SimpleActionClient, SimpleGoalState


class ModSimpleActionClient(SimpleActionClient):
    def __init__(self, ns, ActionSpec):
        super(ModSimpleActionClient, self).__init__(ns, ActionSpec)

    def wait_for_result(self, timeout=rospy.Duration(), interrupt_event=None):
        if not self.gh:
            rospy.logerr("Called wait_for_result when no goal exists")
            return False

        timeout_time = rospy.get_rostime() + timeout
        loop_period = rospy.Duration(0.1)
        with self.done_condition:
            while not rospy.is_shutdown():
                if interrupt_event and interrupt_event.is_set():
                    break

                time_left = timeout_time - rospy.get_rostime()
                if timeout > rospy.Duration(0.0) and time_left <= rospy.Duration(0.0):
                    break

                if self.simple_state == SimpleGoalState.DONE:
                    break

                if time_left > loop_period or timeout == rospy.Duration():
                    time_left = loop_period

                self.done_condition.wait(time_left.to_sec())

        return self.simple_state == SimpleGoalState.DONE

class llmRobotNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        rospy.loginfo("Node {} has been created.".format(name))

        # Create subscriber to get camera message
        self.camera_sub = rospy.Subscriber(
            name="/camera/recognized",
            data_class=String,
            callback=self.camera_callback,
            queue_size=10
        )
        rospy.loginfo("Node has subscriped to '/camera/recognized'")
        self.recognized_object = "" # Initialize recognized object
        
        self.lock = threading.Lock() # Create thread lock 

        # Create service for llm_nlp node
        self.service_name = "/trans"
        self.trans_service = rospy.Service(
            name=self.service_name,
            service_class=trans,
            handler=self.handle_request
        )
        self.parser_success = False
        self.parser_event = threading.Event()

                # Create MoveBaseAction client to control gazebo robot
        self.action_client = ModSimpleActionClient(
            ns = 'move_base',
            ActionSpec = MoveBaseAction
            )
        # Wait for MoveBaseAction server startup
        self.action_client.wait_for_server()
        
    def camera_callback(self, msg):
        """
        callback function to handle the message from '/camera/recognized'

        Args:
            msg (String): the message from '/camera/recoginzed'
        """

        with self.lock:
            self.recognized_object = msg.data
            rospy.loginfo("Recognized object: {}".format(self.recognized_object))

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
            # Ensure the parser is ready
            self.parser_success = False
            self.parser_event.clear()

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
    
    def goal_pose(self, pose):
        goal_pose=MoveBaseGoal()
        goal_pose.target_pose.header.frame_id="map"
        goal_pose.target_pose.pose.position.x=pose[0][0]
        goal_pose.target_pose.pose.position.y=pose[0][1]
        goal_pose.target_pose.pose.position.z=pose[0][2]

        # r, p, y  欧拉角转四元数
        x,y,z,w=tf.transformations.quaternion_from_euler(pose[1][0],pose[1][1],pose[1][2])

        goal_pose.target_pose.pose.orientation.x=x
        goal_pose.target_pose.pose.orientation.y=y
        goal_pose.target_pose.pose.orientation.z=z
        goal_pose.target_pose.pose.orientation.w=w
        
        return goal_pose
        
    def parser_map(self, cmd):
        """
        parser the command from '/trans', then control the robot

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
                command_map[command](parms)
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
        
        patrol_points = [
            [(-4,-3,0.0),(0.0,0.0,100.0)],
            [(4.2,7.4,0.0),(0.0,0.0,180.0)],
            [(5.0,1.0,0.0),(0.0,0.0,180.0)]
        ]

        def patrol():
            while not self.parser_success and not rospy.is_shutdown():
                for pose in patrol_points:
                    if self.parser_success or rospy.is_shutdown():  
                        break
                    goal = self.goal_pose(pose)
                    
                    rospy.loginfo("Sending patrol goal: {}".format(pose))
                    self.action_client.send_goal(goal)
                    finished = self.action_client.wait_for_result(
                        interrupt_event=self.parser_event, 
                    )

                    # 
                    if self.parser_event.is_set():
                        rospy.loginfo("Object found, stopping patrol.")
                        self.action_client.cancel_goal() 
                        break
                    
                    self.action_client.cancel_goal()
                    if not finished:
                        rospy.loginfo("Patrol goal timeout, continuing patrol")

                if not self.parser_success:
                    rospy.loginfo("Object not found, continuing patrol.")
        
        def detect_object(object):
            tf_listener = tf.TransformListener()
            while not self.parser_success and not rospy.is_shutdown():
                if self.recognized_object == object:
                    self.parser_success = True
                    self.parser_event.set()
                    try:
                        # Wait for Transform data
                        tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(3.0))
                        (trans, rot) = tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                        rospy.loginfo("Object found: {} at current position: x: {:.2f}, y: {:.2f}, z: {:.2f}".format(
                            object, trans[0], trans[1], trans[2]
                        ))
                    except Exception as e:
                        rospy.logerr("TF error: {}".format(e))
                    self.action_client.cancel_goal()
                    break
                time.sleep(0.1)
                    
        patrol_thread = threading.Thread(target=patrol)
        patrol_thread.start()
        detect_thread = threading.Thread(target=detect_object, args=(object,))
        detect_thread.start()

        patrol_thread.join()
        detect_thread.join()

        # Add return to origin
        if self.parser_success:
            rospy.loginfo("Returning to origin position.")
            origin_x = rospy.get_param("x", 5.0)
            origin_y = rospy.get_param("y", 1.0)
            origin_z = rospy.get_param("z", 0.0)
            origin_yaw = rospy.get_param("yaw", -3.1)
            origin_pose = self.goal_pose([[origin_x, origin_y, origin_z], [0.0, 0.0, origin_yaw]])
            self.action_client.send_goal(origin_pose)
            finished = self.action_client.wait_for_result()
            if finished:
                rospy.loginfo("Returned to origin successfully.")
            else:
                rospy.logwarn("Return to origin timed out.")
        else:
            rospy.loginfo("Target not found, skipping return-to-origin.")
        

def main():
    node = llmRobotNode(name="llm_robot")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    
if __name__ == "__main__":  
    main()