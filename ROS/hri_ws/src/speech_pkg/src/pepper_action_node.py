#!/usr/bin/python
# coding=utf-8

import rospy
import qi

import actionlib
#from actionlib_msgs import GoalID
from speech_pkg.msg import IntentAction, IntentResult

from lang_settings import AVAILABLE_LANGS
from intents import INTENT_TO_ACTION


class PepperAction(object):

    def __init__(self, name, ip, language):
        self._action_name = name
        self.ip = ip
        self.language = language
        self._as = actionlib.SimpleActionServer(self._action_name, IntentAction, execute_cb=self.execute_cb, auto_start=False)
        self.connect_robot()
        self._as.start()

    def execute_cb(self, message):
        # r = rospy.Rate(1)
        start = rospy.get_time()
        '''Execute goal'''
        #'''
        self.say(out_str=INTENT_TO_ACTION[message.intent][self.language])
        self.move_wheels(intent=message.intent)
        #'''
        duration = rospy.Duration(rospy.get_time()-start)
        rospy.loginfo('%s: Succeeded' % self._action_name)
        result = IntentResult()
        result.time_elapsed = duration
        self._as.set_succeeded(result)
        #print("Arthur Beis is the best.")
    
    def connect_robot(self):
        # Connect to the robot
        print("Connecting to robot...")
        session = qi.Session()
        session.connect('tcp://%s:9559' % self.ip )  # Robot IP
        print("Robot connected")

        self.motion_service = session.service("ALMotion")
        self.motion_service.wakeUp()
        self.motion_service.moveInit()

        #TextToSpeech service
        self.tts = session.service("ALTextToSpeech")
        self.tts.setLanguage("Italian" if self.language == "ita" else "English")
        self.tts.setVolume(0.7)
        self.tts.say("Hola")
    
    def say(self, out_str):
        try:
            self.tts.say(out_str)
        except Exception:
            session = qi.Session()
            session.connect('tcp://%s:9559' % self.ip )
            self.tts = session.service("ALTextToSpeech")
            self.tts.setLanguage("Italian" if self.language == "ita" else "English")
            self.tts.setVolume(0.7)
            self.tts.say(out_str)

    def move_wheels(self, intent):
        try:
            print(INTENT_TO_ACTION[intent]['x'], INTENT_TO_ACTION[intent]['y'], INTENT_TO_ACTION[intent]['theta'])
            self.motion_service.moveTo(0.2, 0.2, 3.14)
            self.motion_service.moveTo(INTENT_TO_ACTION[intent]['x'], INTENT_TO_ACTION[intent]['y'], INTENT_TO_ACTION[intent]["theta"])
        except Exception as e:
            print(e)
            session = qi.Session()
            session.connect('tcp://%s:9559' % self.ip)
            self.motion_service = session.service("ALMotion")
            #self.motion_service.moveTo(0.2, 0.2, 3.14)
            self.motion_service.moveTo(INTENT_TO_ACTION[intent]['x'], INTENT_TO_ACTION[intent]['y'], INTENT_TO_ACTION[intent]["theta"])
            self.tts = session.service("ALTextToSpeech")
            self.tts.setLanguage("Italian" if self.language == "ita" else "English")
            #self.tts.setVolume(0.7)


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