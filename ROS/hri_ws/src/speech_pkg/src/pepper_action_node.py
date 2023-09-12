#!/usr/bin/python
# coding=utf-8

import rospy
import qi

import actionlib
from actionlib_msgs import GoalID
from speech_pkg.msg import IntentAction

from lang_settings import AVAILABLE_LANGS
from intents import INTENT_TO_ACTION


class PepperAction(object):

    def init(self, name:str, ip:str, language:str):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, IntentAction, execute_cb=self.execute_cb, auto_start=False)
        self.connect_robot()
        self.ip = ip
        self.language = language
        self._as.start()

    def execute_cb(self, message):
        # r = rospy.Rate(1)
        start = rospy.get_time()
        '''Execute goal'''
        '''
        self.say(out_str=INTENT_TO_ACTION[self.language])
        self.move_wheels(intent=message.data)
        #'''
        duration = rospy.Duration(rospy.get_time()-start)
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(duration)
        print("Arthur Beis is the best.")
    
    def connect_robot(self):
        # Connect to the robot
        print("Connecting to robot...")
        session = qi.Session()
        session.connect('tcp://%s:9559' % self.ip )  # Robot IP
        print("Robot connected")

        self.motion_service = session.service("ALMotion")
        self.motion_service.wakeUp()

        #TextToSpeech service
        self.tts = session.service("ALTextToSpeech")
        self.tts.setLanguage("Italian" if self.language == "ita" else "English")
        self.tts.setVolume(0)
        self.tts.say("Hello")
    
    def say(self, out_str):
        try:
            self.tts.say(out_str)
        except Exception:
            session = qi.Session()
            session.connect('tcp://%s:9559' % self.ip )
            self.tts = session.service("ALTextToSpeech")
            self.tts.setLanguage("Italian" if self.language == "ita" else "English")
            self.tts.say(out_str)

    def move_wheels(self, intent):
        try:
            self.motion_service.moveTo(x=INTENT_TO_ACTION[intent]['x'], y=INTENT_TO_ACTION[intent]['y'], theta=INTENT_TO_ACTION[intent]["theta"])
        except Exception:
            session = qi.Session()
            session.connect('tcp://%s:9559' % self.ip)
            self.motion_service = session.service("ALMotion")
            self.motion_service.moveTo(x=INTENT_TO_ACTION[intent]['x'], y=INTENT_TO_ACTION[intent]['y'], theta=INTENT_TO_ACTION[intent]["theta"])
            self.tts = session.service("ALTextToSpeech")
            self.tts.setLanguage("Italian" if self.language == "ita" else "English")
            self.tts.setVolume(0.7)


if __name__ == "__main__":
    print("PEPPER ACTION NODE - start")
    LANGUAGE = rospy.get_param("/language")
    PEPPER_IP = rospy.get_param("/pepper_ip")

    if LANGUAGE not in AVAILABLE_LANGS:
        raise Exception("Selected lang not available.\nAvailable langs:", AVAILABLE_LANGS)
    
    rospy.init_node('pepper_action_node')
    pepper = PepperAction(name=rospy.get_name(), ip=PEPPER_IP, language=LANGUAGE)
    rospy.spin()
    print("PEPPER ACTION NODE - end")