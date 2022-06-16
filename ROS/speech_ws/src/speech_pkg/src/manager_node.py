#!/usr/bin/python3

import sys
import rospy
import argparse
from speech_pkg.srv import *

from colorama import Fore
from datetime import datetime
from commands import DEMO3_CMD_ENG, DEMO3_CMD_ITA, DEMO7_CMD_ENG, DEMO7_CMD_ITA
from speech_pkg.msg import Command, Speech


SPEECH_INFO_FILE = '/home/felice/felice/speech-command_interaction/detected_voices/res.txt'
SAVE_SPEECH_INFO = False

command_eng = DEMO7_CMD_ENG
command_ita  = DEMO7_CMD_ITA
speech_counter = 0
robot_listening = False


def publish_cmd(command:int, confidence:float):
    global command_eng, command_ita

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


def run_demo7(req):
    global SPEECH_INFO_FILE, SAVE_SPEECH_INFO, speech_counter, robot_listening, command_eng, command_ita

    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    # print(Fore.MAGENTA + '#'*10 + ' Detected command ' + '#'*10 + '\n{}\n'.format(res) + '#'*38 + Fore.RESET)

    if not robot_listening and res.cmd == 6:
        robot_listening = True
        print(Fore.LIGHTGREEN_EX + '-'*12 + ' ROBOT IS LISTENING ' + '-'*12 + Fore.RESET)

    if robot_listening:
        publish_cmd(command=res.cmd, confidence=res.probs[res.cmd])
        res_str = Fore.CYAN + '#'*10 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*10 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_ita[res.cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n' + '#'* 44 + Fore.RESET + '\n'
        print(res_str)
        
        if SAVE_SPEECH_INFO:
            with open(SPEECH_INFO_FILE, "a") as f:
                f.write(res_str)

    if robot_listening and res.cmd == 5:
        robot_listening = False
        print(Fore.LIGHTRED_EX + '-'*12 + ' ROBOT IS NOT LISTENING ' + '-'*12 + Fore.RESET)

    speech_counter += 1
    # res = speech(res.cmd, res.probs)

    return ManagerResponse(True)    # res.flag


def run_demo3(req):
    global SPEECH_INFO_FILE, SAVE_SPEECH_INFO, speech_counter, robot_listening, command_eng, command_ita

    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    # print(Fore.MAGENTA + '#'*10 + ' Detected command ' + '#'*10 + '\n{}\n'.format(res) + '#'*38 + Fore.RESET)


    publish_cmd(command=res.cmd, confidence=res.probs[res.cmd])
    res_str = Fore.CYAN + '#'*10 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*10 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_ita[res.cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n' + '#'* 44 + Fore.RESET + '\n'
    print(res_str)
        
    if SAVE_SPEECH_INFO:
        with open(SPEECH_INFO_FILE, "a") as f:
            f.write(res_str)

    speech_counter += 1
    # res = speech(res.cmd, res.probs)

    return ManagerResponse(True)    # res.flag


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", required=True, dest="demo", type=int)
    args, unknown = parser.parse_known_args(args=rospy.myargv(argv=sys.argv)[1:])

    pub = rospy.Publisher('/UNISA/SpeechGestureAnalysis/Speech', Speech, queue_size=10)
    
    rospy.init_node('manager')
    
    rospy.wait_for_service('classifier_service')
    # rospy.wait_for_service('speech_service')

    if args.demo == 3:
        command_eng = DEMO3_CMD_ENG
        command_ita  = DEMO3_CMD_ITA
        rospy.Service('manager_service', Manager, run_demo3)
    else:
        command_eng = DEMO7_CMD_ENG
        command_ita  = DEMO7_CMD_ITA
        rospy.Service('manager_service', Manager, run_demo7)
    # rospy.Service('manager_service', Manager, run_demo7)
    # rospy.Service('manager_service', Manager, lambda req: run(req, speech_counter))
    
    classify = rospy.ServiceProxy('classifier_service', Classification)
    print(Fore.GREEN + '\n' + '#'*20 + '\n#   SYSTEM READY   #\n#' + ' '*6 + 'demo {}'.format(args.demo) + ' '*6 + '#\n' + '#'*20 + '\n' + Fore.RESET)

    # speech = rospy.ServiceProxy('speech_service', Talker)

    rospy.spin()