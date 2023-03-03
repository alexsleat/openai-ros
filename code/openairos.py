import os
import openai   
import rospy
from std_msgs.msg import String
import chat_config

class openai_handler:

    def __init__(self, _initial_prompt) -> None:
        openai.api_key = os.getenv("OPENAI_API_KEY")

        self.initial_prompt = _initial_prompt
        self.new_response = False
        self.response = ""
        self.prompt = ""

        # ROS 
        rospy.init_node('openai_handler', anonymous=True)
        self.response_pub = rospy.Publisher('openai/response', String, queue_size=0)
        rospy.Subscriber("openai/prompt", String, self.prompt_sub)
        self.rate = rospy.Rate(10) # 10hz

        self.chat_history = [{"role": "system", "content": self.initial_prompt}]

        # Send initial response when loading for context.
        self.new_response, self.respone = self.get_response(self.initial_prompt)
        # self.response_pub.publish(self.respone)



    # prompt_sub:
    #   When a prompt is recieved, organise it and send a request to the API on demand.      
    def prompt_sub(self, data):

        rospy.loginfo(rospy.get_caller_id() + "OpenAI: Prompt: %s", data.data)
        self.prompt = data.data

        # Log the history
        self.chat_history.append( {"role": "user", "content": self.prompt} )

        # If the chat log is longer than specified, remove the oldest (but not the context)
        if(len(self.chat_history) > chat_config.chat_history_length):
            del self.chat_history[1]

        # Get a response
        self.new_response, self.respone = self.get_response(self.prompt)

    # get_response:
    #   generate and send a request to the openai api, get request and return the useful stuff    
    def get_response(self, _prompt):
        
        success = False
        #################################################################
        # ChatCompletion:
        if chat_config.mode == "ChatCompletion":

            try:
                response = openai.ChatCompletion.create(
                model=chat_config.model,
                messages=self.chat_history,
                max_tokens=chat_config.max_tokens,
                presence_penalty=chat_config.presence_penalty
                )
                
                text_response = response["choices"][0]["message"]["content"]
                self.chat_history.append( {"role": "assistant", "content": text_response} )
                success = True
            except Exception as e:
                rospy.logerr("Error with openai ChatCompletion response")
                text_response = "Sorry, something went wrong. Please try again later."
                success = False

        #################################################################
        # Completion:
        elif chat_config.mode == "Completion":

            # Format the prompt so it replies like a robot in a chat.
            p = self.initial_prompt + " Human:" + _prompt + " " + chat_config.robot_name + ":"  
            p.replace("\n", "")

            try:
                response = openai.Completion.create(
                    model=chat_config.model,
                    prompt=_prompt,
                    temperature=chat_config.temperature,
                    max_tokens=chat_config.max_tokens,
                    top_p=chat_config.top_p,
                    frequency_penalty=chat_config.frequency_penalty,
                    presence_penalty=chat_config.presence_penalty,
                    stop=[" Human:", " " + chat_config.robot_name + ":"]
                )
                text_response = response["choices"][0]["text"].replace("\n", "")
                success = True
            except Exception as e:
                rospy.logerr("Error with openai Completion response")
                text_response = "Sorry, something went wrong. Please try again later."
                success = False

        else:
            text_response = "Sorry, the configuration of the system is wrong. Please contact someone"
            success = False
        
        rospy.loginfo(rospy.get_caller_id() + "OpenAI: Response: %s", text_response)
        
        return success, text_response
    
    # main:
    #   Keep alive, check for new responses and publish the text to the response topic (openai/response)    
    def main(self):

        while not rospy.is_shutdown():

            if(self.new_response):

                rospy.loginfo(self.chat_history)
                # rospy.loginfo(self.respone)
                self.response_pub.publish(self.respone)
                self.new_response = False

            self.rate.sleep()


if __name__ == "__main__":

    # o_h = openai_handler("The following is a chat between a human and a robot called Furhat, who is a helpful and happy at KTH, Stockholm.")
    o_h = openai_handler(chat_config.initial_prompt)
    o_h.main()