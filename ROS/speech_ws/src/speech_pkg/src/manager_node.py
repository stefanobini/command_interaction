#!/usr/bin/python3

import rospy
from speech_pkg.srv import *

from colorama import Fore
from datetime import datetime
from commands import command_eng, command_ita
from speech_pkg.msg import Command, Speech


def publish_cmd(command:int, confidence:float):
    # Make Command message
    cmd_msg = Command()
    cmd_msg.label = command
    cmd_msg.english = command_eng[command]
    cmd_msg.italian = command_ita[command]

    # Make Speech message
    speech_msg = Speech()
    # speech_msg.id = '/cobot1/speech'
    speech_msg.id = 'UNISA.SpeechGestureAnalysis.Speech'
    speech_msg.type = 'Speech'
    # speech_msg.timestamp = Time.now()
    speech_msg.timestamp = datetime.now().isoformat()
    speech_msg.command = cmd_msg
    speech_msg.confidence = float(confidence)

    # print(Fore.LIGHTYELLOW_EX + '#'*30 + '\n' + str(speech_msg) + '\n' + '#'*30 + Fore.RESET)

    #rospy.loginfo(speech_msg)
    pub.publish(speech_msg)


def run(req):
    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    # print(Fore.MAGENTA + '#'*10 + ' Detected command ' + '#'*10 + '\n{}\n'.format(res) + '#'*38 + Fore.RESET)
    cmd, probs = res.cmd, res.probs

    publish_cmd(command=cmd, confidence=probs[cmd])

    res = speech(cmd, probs)

    return ManagerResponse(res.flag)

if __name__ == "__main__":
    pub = rospy.Publisher('/UNISA/SpeechGestureAnalysis/Speech', Speech, queue_size=10)
    
    rospy.init_node('manager')
    
    rospy.wait_for_service('classifier_service')
    rospy.wait_for_service('speech_service')
    rospy.Service('manager_service', Manager, run)
    classify = rospy.ServiceProxy('classifier_service', Classification)
    speech = rospy.ServiceProxy('speech_service', Talker)

    rospy.spin()