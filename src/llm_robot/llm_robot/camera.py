import rclpy, threading
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from roboflow import Roboflow
import cv2, os

class cameraNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Node {} has been created.".format(name))    
        self.camera_publisher_ = self.create_publisher(String, "/camera/recognized", 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.detection = {"class_name": "", "confidence": 0.0}

    def timer_callback(self):
        msg = String()
        if self.detection["class_name"]:
            msg.data = f"Class: {self.detection['class_name']}"
            self.detection["class_name"] = ""
            self.camera_publisher_.publish(msg)
            self.get_logger().info(f"Published to topic '/camera/recognized': '{msg.data}'")

    def update_detection(self, predictions):
        if predictions:
            self.detection["class_name"] = predictions[0]['class']
            self.detection["confidence"] = predictions[0]['confidence']

class camera():
    def __init__(self, camera_node): 
        self.cap = cv2.VideoCapture(0)
        self.frame = None
        self.predictions = []
        self.camera_node = camera_node

        # Roboflow project init
        rf = Roboflow(api_key=os.getenv("ROBOFLOW_API_KEY"))
        project = rf.workspace("buildmyownx").project("bottle-fviyh")
        self.model = project.version(2).model

    def process_frames(self):
        while True:
            ret, self.frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            if self.frame is not None:
                self.predictions = self.model.predict(self.frame, confidence=50, overlap=50).json()['predictions']
                self.camera_node.update_detection(self.predictions)

def main(args=None):
    rclpy.init(args=args)
    camera_node = cameraNode("camera_node")

    try:
        cam = camera(camera_node)
        process_thread = threading.Thread(target=cam.process_frames)
        process_thread.start()
        process_thread.join()
    except Exception as e:
        camera_node.get_logger().error(f"Error: {e}")
        return

    executor = MultiThreadedExecutor()
    executor.add_node(camera_node)

    try:
        executor.spin()
        camera_node.get_logger().info("Init success")   
    except KeyboardInterrupt:
        pass        
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()