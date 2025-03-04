import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from ultralytics import YOLO
import cv2
import os
import threading                
import datetime

class cameraNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Node {} has been created.".format(name))    
        self.camera_publisher_ = self.create_publisher(String, "/camera/recognized", 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.detection = {"class_name": "", "confidence": 0.0}

        self.cap = cv2.VideoCapture(0)
        self.frame = None
        self.predictions = []

        self.model = YOLO("yolov8n.pt")

        # start capture and process threads 
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
        while rclpy.ok():
            if self.frame is not None:
                try:
                    results = self.model.predict(self.frame, conf=0.7, iou=0.5, classes=[39], verbose=False)
                    predictions = []
                    if results and results[0].boxes:
                        boxes = results[0].boxes
                        for i in range(len(boxes)):
                            xywh = boxes.xywh[i].cpu().detach().numpy()
                            conf = float(boxes.conf[i])
                            cls = int(boxes.cls[i])
                            # 获取类别名称
                            class_name = self.model.names[cls] if self.model.names else str(cls)
                            predictions.append({
                                'x': float(xywh[0]),
                                'y': float(xywh[1]),
                                'width': float(xywh[2]),
                                'height': float(xywh[3]),
                                'confidence': conf,
                                'class': class_name
                            })
                    self.predictions = predictions
                    self.update_detection(predictions)
                except Exception as e:
                    self.get_logger.error("Prediction error: %s", e)
    
    def timer_callback(self):
        msg = String()
        if self.detection["class_name"]:
            msg.data = self.detection["class_name"]
            self.detection["class_name"] = ""
            self.camera_publisher_.publish(msg)
            self.get_logger().info("Published to topic '/camera/recognized': '{}'".format(msg.data))

    def update_detection(self, predictions):
        # If there are predictions, update the detection    
        if predictions:
            if predictions[0]['confidence'] and predictions[0]['class']:
                self.detection["class_name"] = predictions[0]['class']
                self.detection["confidence"] = predictions[0]['confidence']
                self.save_frame(predictions)

    def save_frame(self, predictions):
        if self.frame is not None:
            # Create a copy of the frame to draw on
            frame_with_detections = self.frame.copy()

            # Draw bounding boxes and labels on the frame
            for prediction in predictions:
                x = prediction['x']
                y = prediction['y']
                width = prediction['width']
                height = prediction['height']
                confidence = prediction['confidence']
                class_name = prediction['class']

                # Calculate the coordinates of the bounding box
                start_point = (int(x - width / 2), int(y - height / 2))
                end_point = (int(x + width / 2), int(y + height / 2))

                # Draw the bounding box
                cv2.rectangle(frame_with_detections, start_point, end_point, (0, 255, 0), 2)

                # Draw the label above the bounding box
                label = f'{class_name}: {confidence:.2f}'
                cv2.putText(frame_with_detections, label, (start_point[0], start_point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if not os.path.exists('run'):
                os.makedirs('run')
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            filename = "run/{}_{}.jpg".format(class_name, timestamp)
           
            cv2.imwrite(filename, frame_with_detections)
            self.get_logger().info("Frame saved to {}".format(filename))

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