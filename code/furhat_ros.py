# importing the requests library
import requests
import rospy
from std_msgs.msg import String
  

class furhat:

    def __init__(self, ipaddr, port=54321) -> None:

        self.last_say = ""
        self.last_listen = ""
        self.new_listen = False

        # base api-endpoint
        self.URL = "http://" + ipaddr + ":" + str(port) + "/furhat/"

        # ROS 
        rospy.init_node('furhat_handler', anonymous=True)
        self.response_pub = rospy.Publisher('openai/prompt', String, queue_size=0)
        rospy.Subscriber("openai/response", String, self.say_sub)
        self.rate = rospy.Rate(10) # 10hz

    def say_sub(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Furhat: Say: %s", data.data)
        self.prompt = data.data
        self.say(self.prompt)

    def say(self, _data):

        # location given here
        text_speech = _data
        # defining a params dict for the parameters to be sent to the API
        PARAMS = {'text': text_speech} 
        # sending get request and saving the response as response object
        r = requests.post(url = self.URL + "say", params = PARAMS)
        return r

    def listen(self):

        r = requests.get(url = self.URL + "listen")
        return r
    
    def main(self):

        while not rospy.is_shutdown():

            r = self.listen()
            print(r)

            if(self.new_listen):
                rospy.loginfo(self.respone)
                self.response_pub.publish(self.respone)
                self.new_listen = False

            self.rate.sleep()