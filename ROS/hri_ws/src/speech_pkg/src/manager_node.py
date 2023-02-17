#!/usr/bin/python3

import sys
import rospy
from speech_pkg.srv import *
import uuid
from colorama import Fore
from datetime import datetime

from std_msgs.msg import String

from commands_unique_list import DEMO_CMD_ENG, DEMO_CMD_ITA
from speech_pkg.msg import Command, Speech
from demo_utils.post_request import MyRequestPost


SPEECH_INFO_FILE = '/home/felice/speech-command_interaction/acquisition/speech/detected_voices/res.txt'

command_eng = DEMO_CMD_ENG
command_ita  = DEMO_CMD_ITA
speech_counter = 0
robot_listening = False
robot_uuid = uuid.uuid1().node


def publish_cmd(command:int, confidence:float):
    global command_eng, command_ita, robot_uuid

    # Make Command message
    cmd_msg = Command()
    cmd_msg.label = command
    cmd_msg.english = command_eng[command]
    cmd_msg.italian = command_ita[command]

    # Make Speech message
    speech_msg = Speech()
    speech_msg.id = 'UNISA.SpeechGestureAnalysis.Speech:{}'.format(robot_uuid)
    speech_msg.type = 'Speech'
    speech_msg.timestamp = datetime.now().isoformat()
    speech_msg.command = cmd_msg
    speech_msg.confidence = float(confidence)

    # print(Fore.LIGHTYELLOW_EX + '#'*30 + '\n' + str(speech_msg) + '\n' + '#'*30 + Fore.RESET)

    rospy.loginfo(speech_msg)
    pub.publish(speech_msg)


def run_demo7(req):
    global SPEECH_INFO_FILE, speech_counter, robot_listening, command_eng, command_ita, post_request, offset, FIWARE_CB

    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    # print(Fore.MAGENTA + '#'*10 + ' Detected command ' + '#'*10 + '\n{}\n'.format(res) + '#'*38 + Fore.RESET)

    if not robot_listening and res.cmd == 0:
        robot_listening = True
        print(Fore.LIGHTGREEN_EX + '-'*12 + ' ROBOT IS LISTENING ' + '-'*12 + Fore.RESET)

    if robot_listening:
        # publish_cmd(command=res.cmd + offset, confidence=res.probs[res.cmd])
        cmd = res.cmd + offset
        if FIWARE_CB != "None":
            post_request.send_command(command_id=cmd, confidence=res.probs[res.cmd])
        else:
            pub.publish(command_eng[cmd] + " - " + command_ita[cmd])
        res_str = Fore.CYAN + '#'*10 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*10 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_eng[cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_ita[cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n' + '#'* 44 + Fore.RESET + '\n'
        print(res_str)
        
        if rospy.get_param("/save_speech") == True:
            with open(SPEECH_INFO_FILE, "a") as f:
                f.write(res_str)

    if robot_listening and res.cmd == 1:
        robot_listening = False
        print(Fore.LIGHTRED_EX + '-'*12 + ' ROBOT IS NOT LISTENING ' + '-'*12 + Fore.RESET)

    speech_counter += 1
    # res = speech(res.cmd, res.probs)

    return ManagerResponse(True)    # res.flag


def run_demo3(req):
    global SPEECH_INFO_FILE, speech_counter, robot_listening, command_eng, command_ita, post_request, offset

    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    # print(Fore.MAGENTA + '#'*10 + ' Detected command ' + '#'*10 + '\n{}\n'.format(res) + '#'*38 + Fore.RESET)


    # publish_cmd(command=res.cmd, confidence=res.probs[res.cmd])
    # print(res.cmd)
    cmd = res.cmd+6 if res.cmd == 2 else res.cmd    # to manage the unique command list
    post_request.send_command(command_id=cmd, confidence=res.probs[res.cmd])
    res_str = Fore.CYAN + '#'*10 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*10 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_eng[cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_ita[cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n' + '#'* 44 + Fore.RESET + '\n'
    print(res_str)
        
    if rospy.get_param("/save_speech") == True:
        with open(SPEECH_INFO_FILE, "a") as f:
            f.write(res_str)

    speech_counter += 1
    # res = speech(res.cmd, res.probs)

    return ManagerResponse(True)    # res.flag


def run_demo_full(req):
    global SPEECH_INFO_FILE, speech_counter, robot_listening, command_eng, command_ita, post_request, offset, FIWARE_CB

    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    # print(Fore.MAGENTA + '#'*10 + ' Detected command ' + '#'*10 + '\n{}\n'.format(res) + '#'*38 + Fore.RESET)

    if not robot_listening and res.cmd == 23:
        robot_listening = True
        print(Fore.LIGHTGREEN_EX + '-'*12 + ' ROBOT IS LISTENING ' + '-'*12 + Fore.RESET)

    if robot_listening:
        cmd = res.cmd + offset
        # print(cmd)
        if FIWARE_CB != "None":
            post_request.send_command(command_id=cmd, confidence=res.probs[res.cmd])
        else:
            pub.publish(command_eng[cmd] + " - " + command_ita[cmd])
        res_str = Fore.CYAN + '#'*10 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*10 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_eng[cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_ita[cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n' + '#'* 44 + Fore.RESET + '\n'
        print(res_str)
        
        if rospy.get_param("/save_speech") == True:
            with open(SPEECH_INFO_FILE, "a") as f:
                f.write(res_str)

    if robot_listening and res.cmd == 20:
        robot_listening = False
        print(Fore.LIGHTRED_EX + '-'*12 + ' ROBOT IS NOT LISTENING ' + '-'*12 + Fore.RESET)

    speech_counter += 1
    # res = speech(res.cmd, res.probs)

    return ManagerResponse(True)    # res.flag


if __name__ == "__main__":
    DEMO = rospy.get_param("/demo")
    FIWARE_CB = rospy.get_param("/fiware_cb")

    pub = rospy.Publisher('speech_command', String, queue_size=1)
    # pub = rospy.Publisher('/UNISA/SpeechGestureAnalysisAWS/Speech', Speech, queue_size=10)
    # pub = rospy.Publisher('/UNISA/SpeechGestureAnalysisCOBOT/Speech', Speech, queue_size=10)
    
    rospy.init_node('manager')
    
    rospy.wait_for_service('classifier_service')
    # rospy.wait_for_service('speech_service')

    command_eng = DEMO_CMD_ENG
    command_ita  = DEMO_CMD_ITA

    if DEMO == 3:
        offset = 0
        rospy.Service('manager_service', Manager, run_demo3)
    elif DEMO == 7:
        offset = 2
        rospy.Service('manager_service', Manager, run_demo7)
    elif DEMO == 0:
        offset = 0
        rospy.Service('manager_service', Manager, run_demo_full)
    # rospy.Service('manager_service', Manager, run_demo7)
    # rospy.Service('manager_service', Manager, lambda req: run(req, speech_counter))

    if FIWARE_CB != "None":
        post_request = MyRequestPost(robot_uuid, entity="UNISA.SpeechGestureAnalysis.Speech", msg_type="Speech", address=FIWARE_CB, port=1026)
        post_request.create_entity()
    
    classify = rospy.ServiceProxy('classifier_service', Classification)
    print(Fore.GREEN + '\n' + '#'*20 + '\n#   SYSTEM READY   #\n#' + ' '*6 + 'demo {}'.format(DEMO) + ' '*6 + '#\n' + '#'*20 + '\n' + Fore.RESET)

    # speech = rospy.ServiceProxy('speech_service', Talker)

    rospy.spin()