# importing the requests library
import requests, json

import time

import rospy
from std_msgs.msg import String
  

class furhat:

    def __init__(self, ipaddr, port=54321) -> None:

        self.last_say = ""
        self.last_listen = ""
        self.new_listen = False

        self.speaking_counter = 0

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
        self.listen_stop()
        r = requests.post(url = self.URL + "say", params = PARAMS)
        self.speaking_counter = len(text_speech)
        print("Counter val: ", self.speaking_counter)

        return r

    def listen(self):

        print("Listening - started")
        r = requests.get(url = self.URL + "listen")
        print("Listening - stopped")

        json_data = json.loads(r.text)
        msg = json_data["message"]

        if(len(msg) > 0):
            rospy.loginfo(json_data)

        return r, msg
    
    def listen_stop(self):

        print("Listening - force quit")
        r = requests.post(url = self.URL + "listen/stop")
        return r        
    
    def gesture(self, _data):

        # location given here
        text_gesture = _data
        # defining a params dict for the parameters to be sent to the API
        PARAMS = {'name': text_gesture} 
        # sending get request and saving the response as response object
        r = requests.post(url = self.URL + "gesture", params = PARAMS)
        return r        
    
    def main(self):

        while not rospy.is_shutdown():

            if self.speaking_counter <= 0:

                r, msg = self.listen()
                if(len(msg) > 0):
                    self.response_pub.publish(msg)

            else:
                self.speaking_counter = self.speaking_counter - 1

                # Do some gestures when ready to listen again, to signal timing
                if(self.speaking_counter == 2):
                    self.gesture("BrowRaise")

            self.rate.sleep()


if __name__ == "__main__":

    fh = furhat("192.168.100.27", 54321)
    fh.main()