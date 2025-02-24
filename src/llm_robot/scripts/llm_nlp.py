#!/usr/bin/env python

import rospy, os, threading
from llm_robot.srv import trans, transRequest 
from openai import OpenAI

SILICONFLOW_API_KEY = os.getenv("SILICONFLOW_API_KEY")
PROMPT = """
    You are the most accurate command parser! Your task is to transfer information between human and robot.
    
    CLASS_NAME = (
        'mask',
        'phone',
        'bottle',
        'gloves',
        'metal',
        'palstic bag',
        'sunglasses',
        'boll',
        'door',
        'other',
    )

    # Instructions
    - First, Abstracting actionable words from conversations.
    - Second, Compare the object to the object in CLASS_NAME and choose the word with the closest meaning to replace it.
    - Finally, Generate a json string.

    # Example
    conversation: find a garbage bag.
    answer:{
        "commands":[
            {
                "command": "find",
                "parms":{
                    "object":"palstic bag"
                }
            }
        ]
    }

    conversation: find my Coke.
    answer:{
        "commands":[
            {
                "command": "find",
                "parms":{
                    "object":"bottle"
                }
            }
        ]
    }

    conversation: pick up my phone and close the door.
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

        self.service_name = "/trans"

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
                model="deepseek-ai/DeepSeek-V3",
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
        rospy.loginfo("Waiting for service: {}".format(self.service_name))
        rospy.wait_for_service(self.service_name)
        try:
            command_client = rospy.ServiceProxy(
                name=self.service_name,
                service_class=trans
                )
        except rospy.ServiceException as e:
            rospy.logerr("Failed to create service proxy: {}".format(e))
            return 
        
        while not rospy.is_shutdown():
            user_input = input("Enter a command: ")
            result = self.genai(user_input)

            # Crafting service request
            request = transRequest()
            request.command = result

            try:
                rospy.loginfo("Service call: \n{}".format(request.command))
                response = command_client(request) 
                if response.is_success:
                    rospy.loginfo("Service call succeeded: {}".format(response))
                else:
                    rospy.logwarn("Opps! Something went wrong")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))

def main():
    nlp_node = NLPNode("nlp_node")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")   

if __name__ == "__main__":
    main()
    
