import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from roboflow import Roboflow
import cv2
import os
import threading

class cameraNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Node {} has been created.".format(name))    
        self.camera_publisher_ = self.create_publisher(String, "/camera/recognized", 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.detection = {"class_name": "", "confidence": 0.0}

        # 打开摄像头
        self.cap = cv2.VideoCapture(0)
        self.frame = None
        self.predictions = []

        # Roboflow project init
        rf = Roboflow(api_key=os.getenv("ROBOFLOW_API_KEY"))
        project = rf.workspace("buildmyownx").project("bottle-fviyh")
        self.model = project.version(2).model

        # 启动线程
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.start()
        self.process_thread = threading.Thread(target=self.process_frames)
        self.process_thread.start()

    def capture_frames(self):
        while True:
            ret, self.frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to grab frame")
                break

    def process_frames(self):
        while True:
            if self.frame is not None:
                self.predictions = self.model.predict(self.frame, confidence=50, overlap=50).json()['predictions']
                self.update_detection(self.predictions)

    def timer_callback(self):
        msg = String()
        if self.detection["class_name"]:
            msg.data = "Class: {}".format(self.detection["class_name"]) 
            self.detection["class_name"] = ""
            self.camera_publisher_.publish(msg)
            self.get_logger().info("Published to topic '/camera/recognized': '{}'".format(msg.data))

    def update_detection(self, predictions):
        if predictions:
            self.detection["class_name"] = predictions[0]['class']
            self.detection["confidence"] = predictions[0]['confidence']

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