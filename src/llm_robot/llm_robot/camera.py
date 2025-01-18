import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from inference import InferencePipeline
import cv2, os

class cameraNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Node {} has been created.".format(name))    
        self.camera_publisher_ = self.create_publisher(msg_type=String, topic="/camera/recognized", qos_profile=10)
        self.timer_ = self.create_timer(0.3, self.timer_callback)
        self.detection = {"class_name": [], "confidence": []}
    
    def timer_callback(self):
        msg = String()
        # msg.data = "bottle"
        if self.detection["class_name"]:
            msg.data = self.detection["class_name"]
            self.camera_publisher_.publish(msg)
            self.get_logger().info("Published to topic '/camera/recognized': '{}'".format(msg.data))
    
    def my_sink(self, result, video_frame):
        if result.get("output_image"):  
            cv2.waitKey(1)

        predictions = result.get("predictions")
        if predictions and len(predictions.confidence) > 0 and len(predictions.data["class_name"]) > 0 and predictions.confidence[0] > 0.5:
            self.detection["class_name"] = predictions.data["class_name"][0]
            self.detection["confidence"] = predictions.confidence[0]

def main(args=None):
    rclpy.init(args=args)
    camera_node = cameraNode("camera_node")

    # pipeline object init
    try:
        pipeline = InferencePipeline.init_with_workflow(
            api_key=os.getenv("ROBOFLOW_API_KEY"),
            workspace_name="buildmyownx",
            workflow_id="custom-workflow",
            video_reference=1, # Path to video, device id (int, usually 0 for built in webcams), or RTSP stream url
            max_fps=30,
            on_prediction=camera_node.my_sink
        )
    except Exception as e:
        camera_node.get_logger().error("Error: {}".format(e))
        return

    executor = MultiThreadedExecutor()
    executor.add_node(camera_node)

    try:
        executor.spin()
        pipeline.start()
        pipeline.join()
    except KeyboardInterrupt:
        pass        
    finally:
        camera_node.destroy_node()
        pipeline.terminate()
        rclpy.shutdown()

if __name__ == "__main__":
    main()