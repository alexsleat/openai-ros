import os
import openai
import rospy
from std_msgs.msg import String

class openai_handler:

    def __init__(self, _initial_prompt) -> None:
        openai.api_key = os.getenv("OPENAI_API_KEY")

        self.initial_prompt = _initial_prompt
        self.new_response = False
        self.response = ""
        self.prompt = ""
        self.chat_history = self.initial_prompt
        self.robot_name = "AI"
        self.chat_history_flag = True

        self.openai_model = "text-curie-001"

        # Send initial response when loading for context.
        self.new_response, self.respone = self.get_response(self.initial_prompt)

        # ROS 
        rospy.init_node('openai_handler', anonymous=True)
        self.response_pub = rospy.Publisher('openai/response', String, queue_size=0)
        rospy.Subscriber("openai/prompt", String, self.prompt_sub)
        self.rate = rospy.Rate(10) # 10hz
    
    def prompt_sub(self, data):
        rospy.loginfo(rospy.get_caller_id() + "OpenAI: Prompt: %s", data.data)
        self.prompt = data.data
        self.chat_history = self.chat_history + " Human:" + self.prompt
        self.new_response, self.respone = self.get_response(self.prompt)

    def get_response(self, _prompt):
        
        success = False
        p = ""
        if(self.chat_history_flag):
            p = self.chat_history + " " + self.robot_name + ":"  
        else:
            p = self.initial_prompt + " Human:" + _prompt + " " + self.robot_name + ":"  
        p.replace("\n", "")

        print("Prompt: ", p)
        response = openai.Completion.create(
            model=self.openai_model,
            prompt=p,
            temperature=0.5,
            max_tokens=256,
            top_p=0.3,
            frequency_penalty=0.1,
            presence_penalty=0,
            stop=[" Human:", " " + self.robot_name + ":"]
        )
        success = True
        
        text_response = response["choices"][0]["text"].replace("\n", "")
        rospy.loginfo(rospy.get_caller_id() + "OpenAI: Response: %s", text_response)
        self.chat_history = self.chat_history + " " + self.robot_name + ":" + text_response

        return success, text_response
    
    def main(self):

        while not rospy.is_shutdown():

            if(self.new_response):
                rospy.loginfo(self.respone)
                self.response_pub.publish(self.respone)
                self.new_response = False

            self.rate.sleep()


if __name__ == "__main__":

    # o_h = openai_handler("The following is a chat between a human and a robot called Furhat, who is a helpful and happy at KTH, Stockholm.")
    o_h = openai_handler("A conversation between a human and an AI who is assertive sassy, sarcastic that likes to lie and make fun of the human. It also likes to ask questions.")
    o_h.main()