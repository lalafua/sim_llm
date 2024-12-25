import google.generativeai as genai
import rclpy, os
from rclpy.node import Node

class 


def main(args=None):
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    rclpy.init(args=args)
    genai.configure(api_key=GEMINI_API_KEY)
    model = genai.GenerativeModel("gemini-1.5-flash")
    response = model.generate_content("explain how ai works")
    node = Node("nlp_llm")
    node.get_logger().info(response.text)
    rclpy.spin(node)
    node.shutdown()