#!/usr/bin/env python

import rospy, os, threading
from llm_robot.srv import trans 
from openai import OpenAI

SILICONFLOW_API_KEY = os.getenv("SILICONFLOW_API_KEY")
PROMPT = """
    prompt: Hello DeepseekV3! You will receive a text command from a robot. You need to convert it to a json string in plain text.
    prompt: find the plastic bottle.
        answer:{
            "commands":[
                {
                    "command": "find",
                    "parms":{
                        "object":"plastic bottle"
                    }
                }
            ]
        }
    prompt: pick up the red boll.
        answer:{
            "commands":[
                {
                    "command": "pick_up",
                    "parms":{
                        "object":"red boll"
                    }
                }
            ]
        }
    prompt: pick up my phone and close the door.
        answer:{
            "commands":[
                {
                    "command": "pick_up",
                    "parms":{
                        "object": "phone"
                    }
                },
                {
                    "command": "close",
                    "parms":{
                        "object": "door"
                    }
                }
            ]
        }
"""

class NLPNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        rospy.loginfo("Node {} has been created.".format(name))

        self.service_name = "/llm_nlp/cmd"

        self.siliconflow_client = OpenAI(api_key=SILICONFLOW_API_KEY, base_url="https://api.siliconflow.cn/v1")

        # Start a new thread to wait for user input
        self.input_thread = threading.Thread(target=self.wait_for_input)
        self.input_thread.daemon = True
        self.input_thread.start()
        
    def genai(self, msg):
        """
        Calls the deepseek model to generate a response (a JSON string)
        """

        response = self.siliconflow_client.chat.completions.create(
                model="deepseek-ai/DeepSeek-R1-Distill-Qwen-7B",
                messages=[
                    {
                        "role": "system", 
                        "content": PROMPT,
                    },
                    {
                        "role": "user", 
                        "content": msg,
                    },
                ],
                stream=False
            )
        
        response_content = response.choices[0].message.content
        response_content = response_content.replace("```json", "").replace("```", "").strip()
        return response_content
    
    def wait_for_input(self):
        """
        Waits for user input, calls the deepseek model, then sends the result to a service request.
        """
        
        # Wait for the service to be available
        rospy.wait_for_service(self.service_name)
        try:
            command_client = rospy.ServiceProxy(self.service_name, trans)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to create service proxy: {}".format(e))
            return 
        
        while not rospy.is_shutdown():
            user_input = input("Enter a command: ")
            result = self.genai(user_input)

            # Crafting service request
            request = trans.CommandRequest()
            request.command = result

            try:
                response = command_client(request)
                if response.is_success:
                    rospy.loginfo("Service call succeeded: {}".format(response))
                else:
                    rospy.logwarn("Opps! Something went wrong")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))

def main():
    nlp_node = NLPNode("nlp_node")
    rospy.spin()

if __name__ == "__main__":
    main()
    
