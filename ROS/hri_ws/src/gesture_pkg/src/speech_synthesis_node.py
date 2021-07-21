#!/usr/bin/env python
from pepper_utils import Pepper
from settings import demo_settings

import rospy
from std_msgs.msg import String

import re

class SpeechSynthesisNode:
    '''SpeechSynthesisNode implements a ROS interface for a text to speech engine.

    The node is subscribed to the followg topics:

    - **text_2_speech** : String representing the sentence to say

    The available methods are:
    
    - **\_\_init\_\_(self)**: constructor
    - **say(self, data):**: ros topic callback
    - **start(self)**: starts the ros node instance
    '''
    def __init__(self):
        self.pepper_robot = Pepper(demo_settings.pepper.ip,demo_settings.pepper.port)
    
    def say(self, data):
        sentence = data.data
        #sentence = re.sub(',', "\\\\pau={}\\\\".format(demo_settings.pepper.speech.comma_pause), data.data)
        #sentence = re.sub('.', "\\\\pau={}\\\\".format(demo_settings.pepper.speech.period_pause), sentence)

        self.pepper_robot.say(
            sentence,
            demo_settings.pepper.speech.animated, 
            demo_settings.pepper.speech.rspd,
            demo_settings.pepper.speech.language
        )

    def start(self):
        rospy.init_node('speech_synthesis_node', anonymous=True)
        sub = rospy.Subscriber("text_2_speech", String, self.say)

        rospy.spin()

if __name__ == "__main__":
    try:
        image_input = SpeechSynthesisNode()
        image_input.start()
    except rospy.ROSInterruptException:
        pass