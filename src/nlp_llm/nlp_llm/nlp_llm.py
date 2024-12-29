import google.generativeai as genai
import rclpy, os
from rclpy.node import Node
import absl.logging

# Initialize the logger
absl.logging.set_verbosity(absl.logging.INFO)
absl.logging.use_absl_handler()

class NLPNode():
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    PROMPT = """
        prompt: Hello Gemini! You will receive a text command from a robot. You need to convert it to a json string."
        prompt: find the plastic bottle.
            answer:[
                "commands":{
                    "command": "find",
                    "goal":"plastic bottle"
                }
            ]
        prompt: find the red boll.
            answer:[
                "commands":{
                    "command": "find",
                    "goal":"red boll"
                }
            ]
        prompt: find the phone.
            answer:[
                "commands":{
                    "command": "find",
                    "goal":"phone"
                }
            ]
    """

    def __init__(self):       
        genai.configure(api_key=self.GEMINI_API_KEY)
        self.model = genai.GenerativeModel("gemini-1.5-flash")

    def gem_genai(self, prompt):
        response = self.model.generate_content(
            contents=prompt,
            generation_config = genai.GenerationConfig(
                max_output_tokens = 500,
                temperature = 0.1,
            )
        )
        return response.text

def main(args=None):
    nlp_node = NLPNode()
    while True:
        myinput = nlp_node.PROMPT + input("Enter a command: ")
        completion = nlp_node.gem_genai(myinput)
        print(completion)

if __name__ == "__main__":
    main()

# def main(args=None):
#     GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
#     rclpy.init(args=args)
#     genai.configure(api_key=GEMINI_API_KEY)
#     model = genai.GenerativeModel("gemini-1.5-flash")
#     response = model.generate_content("explain how ai works")
#     node = Node("nlp_llm")
#     node.get_logger().info(response.text)
#     rclpy.spin(node)
#     node.shutdown()