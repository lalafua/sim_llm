#!/usr/bin/env python
# filepath: /home/lalafua/myWorkSpace/ros1_sim_llm/src/llm_robot/scripts/camera.py

import rospy
from std_msgs.msg import String
import cv2
import os
import threading                
import datetime
from roboflow import Roboflow

class CameraNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        rospy.loginfo("Node {} has been created.".format(name))
        
        # Create publisher to publish recognized object
        self.camera_pub = rospy.Publisher("/camera/recognized", String, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        self.detection = {"class_name": "", "confidence": 0.0}

        self.cap = cv2.VideoCapture(0)
        self.frame = None
        self.predictions = []

        # Init Roboflow project 
        rf = Roboflow(api_key=os.getenv("ROBOFLOW_API_KEY"))
        project = rf.workspace("buildmyownx").project("bottle-fviyh")
        self.model = project.version(2).model

        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.daemon = True
        self.capture_thread.start()

        self.process_thread = threading.Thread(target=self.process_frames)
        self.process_thread.daemon = True
        self.process_thread.start()

    def capture_frames(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Failed to grab frame")
                rospy.sleep(0.1)
                continue
            self.frame = frame

    def process_frames(self):
        while not rospy.is_shutdown():
            if self.frame is not None:
                try:
                    self.predictions = self.model.predict(self.frame, confidence=70, overlap=50).json()['predictions']
                    self.update_detection(self.predictions)
                except Exception as e:
                    rospy.logerr("Prediction error: %s", e)
            rospy.sleep(0.1)

    def timer_callback(self, event):
        if self.detection["class_name"]:
            msg = String()
            msg.data = self.detection["class_name"]
            self.detection["class_name"] = ""
            self.camera_pub.publish(msg)
            rospy.loginfo("Published to '/camera/recognized': '{}'".format(msg.data))

    def update_detection(self, predictions):
        if predictions:
            if 'confidence' in predictions[0] and 'class' in predictions[0]:
                self.detection["class_name"] = predictions[0]['class']
                self.detection["confidence"] = predictions[0]['confidence']
                self.save_frame(predictions)

    def save_frame(self, predictions):
        if self.frame is not None:
            frame_with_detections = self.frame.copy()

            for prediction in predictions:
                x = prediction['x']
                y = prediction['y']
                width = prediction['width']
                height = prediction['height']
                confidence = prediction['confidence']
                class_name = prediction['class']

                start_point = (int(x - width / 2), int(y - height / 2))
                end_point = (int(x + width / 2), int(y + height / 2))

                cv2.rectangle(frame_with_detections, start_point, end_point, (0, 255, 0), 2)
                label = "{}: {:.2f}".format(class_name, confidence)
                cv2.putText(frame_with_detections, label, (start_point[0], start_point[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if not os.path.exists('run'):
                os.makedirs('run')
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            class_name = predictions[0]['class']
            filename = "run/{}_{}.jpg".format(class_name, timestamp)
            cv2.imwrite(filename, frame_with_detections)
            rospy.loginfo("Frame saved to {}".format(filename))

def main():
    node = CameraNode("camera_node")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down camera_node")
    finally:
        if node.cap.isOpened():
            node.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()