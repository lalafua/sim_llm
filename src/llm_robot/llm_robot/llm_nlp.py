import google.generativeai as genai
import rclpy, os, threading
import absl.logging
from std_msgs.msg import String
from rclpy.node import Node

# Initialize the absl logger for gemini
absl.logging.set_verbosity(absl.logging.INFO)
absl.logging.use_absl_handler()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
PROMPT = """
    prompt: Hello Gemini! You will receive a text command from a robot. You need to convert it to a json string."
    prompt: find the plastic bottle.
        answer:{
            "commands":[{
                "command": "find",
                "parms":{
                    "goal":"red boll",
                    }
            },
            ]
        }
    prompt: find the red boll.
        answer:{
            "commands":[{
                "command": "find",
                "parms":{
                    "goal":"red boll",
                    }
                }
            },
            ]
        }
    prompt: find the phone.
        answer:{
            "commands":[{
                "command": "find",
                "parms":{
                    "goal":"phone",
                    }
                }
            },
            ]
        }
"""

class NLPNode(Node):
    def __init__(self, name):
        # Initialize the node
        super().__init__(name)
        self.get_logger().info("Node {} has been created.".format(name))      
        self.command_publisher_ = self.create_publisher(msg_type=String, topic="nlp", qos_profile=10)
        self.get_logger().info("Topic nlp has been created.")
        self.msg = String()

        # Initialize the gemini model
        genai.configure(api_key=GEMINI_API_KEY)
        self.model = genai.GenerativeModel("gemini-1.5-flash")

         # Start a thread to wait for user input
        self.input_thread = threading.Thread(target=self.wait_for_input)
        self.input_thread.daemon = True
        self.input_thread.start()
    
    def gem_genai(self, prompt):
        """
        This function uses the gemini model to generate a response to a prompt.
        
        parameters: prompt: str
        return: response: str
        """

        response = self.model.generate_content(
            contents=prompt,
            generation_config = genai.GenerationConfig(
                max_output_tokens = 500,
                temperature = 0.1,
            )
        )
        return response.text
    
    def wait_for_input(self):
        """
        This function waits for user input and publishes the input to the nlp topic.
        """

        while rclpy.ok():
            user_input = input("Enter a command: ")
            user_input = PROMPT + user_input
            self.msg.data = self.gem_genai(user_input)
            self.command_publisher_.publish(self.msg)
            self.get_logger().info("Data has been published to the topic nlp: \n{}".format(self.msg.data))


def main(args=None):
    rclpy.init(args=args)
    nlp_node = NLPNode("nlp_llm")
    rclpy.spin(nlp_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
