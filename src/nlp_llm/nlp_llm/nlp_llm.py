import google.generativeai as genai
import rclpy, os
import absl.logging
import json
import std_msgs

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

class NLPNode(rclpy.Node):
    def __init__(self, name):
        super().__init__(name)       
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
        
        try:
            json_response = json.loads(response.text)
            return json_response
        except json.JSONDecodeError:
            
            return None

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