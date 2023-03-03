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
        self.rate = rospy.Rate(12) # 10hz

    # Callback for response subscriber. 
    #   Aka gets details from openai api and speaks it out on the robot
    def say_sub(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Furhat: Say: %s", data.data)
        self.prompt = data.data
        self.say(self.prompt)

    # Say:
    #   Send the post request for the say command via the Remote API
    def say(self, _data):

        # defining a params dict for the parameters to be sent to the API
        PARAMS = {'text': _data} 
        # sending get request and saving the response as response object
        self.listen_stop()
        r = requests.post(url = self.URL + "say", params = PARAMS)
        # As an estimate, set the timer to the same length of the text speech.
        self.speaking_counter = len(_data)

        return r
    # Listen:
    #   Send the get request for the listen command via the Remote API
    def listen(self):

        # Set LED to green while listening:
        self.led(0,255,0)
        # Call the (blocking) listen
        r = requests.get(url = self.URL + "listen")
        # Set LED to white while not listening:
        self.led(0,0,0)
        # Conver to JSON, get the text
        json_data = json.loads(r.text)
        msg = json_data["message"]
        # Log if the msg wasn't empty
        if(len(msg) > 0):
            rospy.loginfo(json_data)

        return r, msg

    # listen_stop:
    #   Send the post request for the listening stop command via the Remote API    
    def listen_stop(self):

        # Turn LEDs white when force stop called
        self.led(255,255,255)
        r = requests.post(url = self.URL + "listen/stop")
        return r        

    # gesture:
    #   Send the post request for the gesture command via the Remote API        
    def gesture(self, _data):

        # location given here
        text_gesture = _data
        # defining a params dict for the parameters to be sent to the API
        PARAMS = {'name': text_gesture} 
        # sending get request and saving the response as response object
        r = requests.post(url = self.URL + "gesture", params = PARAMS)
        return r        

    # led:
    #   Send the post request for the led command via the Remote API     
    def led(self, _red=0, _green=0, _blue=0):

        # defining a params dict for the parameters to be sent to the API
        PARAMS = {'red': _red,
                  'green': _green,
                  'blue': _blue} 
        
        # sending get request and saving the response as response object
        r = requests.post(url = self.URL + "led", params = PARAMS)
        return r    
    
    # main:
    #   Keep alive, check the listen command and publish the text to the response topic (openai/prompt)
    def main(self):

        while not rospy.is_shutdown():

            # Check if speaking, if not listen to the human
            if self.speaking_counter <= 0:

                # Listen and if the result isn't empty, publish it
                r, msg = self.listen()
                if(len(msg) > 0):
                    # Set LED to blue when detected something:
                    self.led(0,0,255)
                    self.speaking_counter = 10
                    self.response_pub.publish(msg)

            # If the robot is speaking, keep waiting for the counter to finish
            else:
                self.speaking_counter = self.speaking_counter - 1
                self.led(0,0,255)

            # sleep to fit hz, won't work during listening mode because the blocking function.
            self.rate.sleep()


if __name__ == "__main__":

    fh = furhat("192.168.100.27", 54321)
    fh.main()