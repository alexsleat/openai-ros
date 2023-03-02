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

        self.new_response, self.respone = self.get_response(self.initial_prompt)

        rospy.init_node('openai_handler', anonymous=True)

        self.response_pub = rospy.Publisher('openai/response', String, queue_size=0)
        rospy.Subscriber("openai/prompt", String, self.prompt_sub)

        self.rate = rospy.Rate(10) # 10hz
    
    def prompt_sub(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.prompt = data.data
        self.new_response, self.respone = self.get_response(self.prompt)

    def get_response(self, _prompt):
        
        success = False
        try:
            response = openai.Completion.create(
                model="text-davinci-003",
                prompt=_prompt,
                temperature=0.5,
                max_tokens=60,
                top_p=0.3,
                frequency_penalty=0.5,
                presence_penalty=0
            )
            success = True
        except Exception as e:
            rospy.logerr("A problem occured with the request: ", e)

        return success, response
    
    def main(self):

        while not rospy.is_shutdown():

            if(self.new_response):
                rospy.loginfo(self.respone)
                self.response_pub.publish(self.respone)
                self.new_response = False

            self.rate.sleep()


if __name__ == "__main__":

    o_h = openai_handler("You are a humanoid robot called Pepper, and are happy to talk with humans about anything.")
    o_h.main()