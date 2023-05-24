#!/usr/bin/python3

import sys
import rospy
from speech_pkg.srv import *
import uuid
from colorama import Fore
from datetime import datetime
import time

from std_msgs.msg import String

from commands import DEMO3_CMD_ENG, DEMO3_CMD_ITA, DEMO7_CMD_ENG, DEMO7_CMD_ITA, DEMO7P_CMD_ENG, DEMO7P_CMD_ITA, DEMO_CMD_ENG, DEMO_CMD_ITA
# from commands_unique_list import DEMO_CMD_ENG, DEMO_CMD_ITA
from speech_pkg.msg import Command, Speech
from demo_utils.post_request import MyRequestPost


SPEECH_INFO_FILE = '/home/felice/command_interaction/ROS/detected_voices/res.txt'
speech_counter = 0
robot_listening = False
robot_uuid = uuid.uuid1().node
START_THRESHOLD = 0.03


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
    print(Fore.MAGENTA + '#'*10 + ' Detected command ' + '#'*10 + '\n{}\n{:.3f}/{:.3f}\n'.format(res.cmd, res.probs[res.cmd], res.probs[0]) + '#'*38 + Fore.RESET)

    if not robot_listening and (res.cmd == 0 or (res.probs[0]>START_THRESHOLD and res.cmd == len(command_eng))):
        robot_listening = True
        print(Fore.LIGHTGREEN_EX + '-'*12 + ' ROBOT IS LISTENING ' + '-'*12 + Fore.RESET)

    if robot_listening:
        # publish_cmd(command=res.cmd + offset, confidence=res.probs[res.cmd])
        cmd = res.cmd + offset
        if res.cmd == len(DEMO7_CMD_ENG)-1:
            cmd = len(DEMO3_CMD_ENG) - 1
        cb_reply_time = time.time()
        if FIWARE_CB == "None":
            pub.publish(command_eng[cmd] + " - " + command_ita[cmd])
        elif res.cmd != len(command_eng)-1:
            post_request.send_command(command_id=cmd, confidence=res.probs[res.cmd])
        cb_reply_time = time.time() - cb_reply_time
        print("COMUNICATION TIME: {:.4f} s".format(cb_reply_time))
        res_str = Fore.CYAN + '#'*6 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*6 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_eng[res.cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_ita[res.cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n' + '#'* 44 + Fore.RESET + '\n'
        #print(res_str)
        
        if rospy.get_param("/save_speech") == True:
            with open(SPEECH_INFO_FILE, "w") as f:
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
    #print(Fore.MAGENTA + '#'*10 + ' Detected command ' + '#'*10 + '\n{}\n'.format(res) + '#'*38 + Fore.RESET)


    # publish_cmd(command=res.cmd, confidence=res.probs[res.cmd])
    # print(res.cmd)
    #cmd = res.cmd+6 if res.cmd == 2 else res.cmd    # to manage the unique command list
    cmd = res.cmd
    cb_reply_time = time.time()
    if FIWARE_CB == "None":
        pub.publish(command_eng[cmd] + " - " + command_ita[cmd])
    elif cmd != len(command_eng)-1:
        post_request.send_command(command_id=cmd, confidence=res.probs[res.cmd])
    cb_reply_time = time.time() - cb_reply_time
    print("COMUNICATION TIME: {:.4f} s".format(cb_reply_time))
    res_str = Fore.CYAN + '#'*10 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*10 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_eng[cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_ita[cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n' + '#'* 44 + Fore.RESET + '\n'
    print(res_str)
        
    if rospy.get_param("/save_speech"):
        with open(SPEECH_INFO_FILE, "a") as f:
            f.write(res_str)

    speech_counter += 1
    # res = speech(res.cmd, res.probs)

    return ManagerResponse(True)    # res.flag


def run_demo_full(req):
    global SPEECH_INFO_FILE, speech_counter, robot_listening, command_eng, command_ita, post_request, offset, FIWARE_CB

    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    print(Fore.MAGENTA + '#'*10 + ' Detected command ' + '#'*10 + '\n{}\n{:.3f}/{:.3f}\n'.format(res.cmd, res.probs[res.cmd], res.probs[0]) + '#'*38 + Fore.RESET)

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
    LANG = rospy.get_param("/language")
    FIWARE_CB = rospy.get_param("/fiware_cb")

    pub = rospy.Publisher('speech_command', String, queue_size=1)
    # pub = rospy.Publisher('/UNISA/SpeechGestureAnalysisAWS/Speech', Speech, queue_size=10)
    # pub = rospy.Publisher('/UNISA/SpeechGestureAnalysisCOBOT/Speech', Speech, queue_size=10)
    
    rospy.init_node('manager')
    
    rospy.wait_for_service('classifier_service')
    # rospy.wait_for_service('speech_service')

    if DEMO == str(3):
        offset = 0
        command_eng = DEMO3_CMD_ENG
        command_ita = DEMO3_CMD_ITA
        rospy.Service('manager_service', Manager, run_demo3)
    elif DEMO == str(7):
        offset = 4
        command_eng = DEMO7_CMD_ENG
        command_ita = DEMO7_CMD_ITA
        rospy.Service('manager_service', Manager, run_demo7)
    elif DEMO == "7_plus":
        offset = 4
        command_eng = DEMO7P_CMD_ENG
        command_ita = DEMO7P_CMD_ITA
        rospy.Service('manager_service', Manager, run_demo7)
    elif DEMO == "full":
        offset = 0
        command_eng = DEMO_CMD_ENG
        command_ita = DEMO_CMD_ITA
        rospy.Service('manager_service', Manager, run_demo_full)
    # rospy.Service('manager_service', Manager, run_demo7)
    # rospy.Service('manager_service', Manager, lambda req: run(req, speech_counter))

    if FIWARE_CB != "None":
        post_request = MyRequestPost(robot_uuid, entity="UNISA.SpeechGestureAnalysis.Speech", msg_type="Speech", address=FIWARE_CB, port=1026)
        post_request.create_entity()
    
    classify = rospy.ServiceProxy('classifier_service', Classification)
    print(Fore.GREEN + '\n' + '#'*24 + '\n#     SYSTEM READY     #\n#' + ' '*6 + 'demo {} {}'.format(DEMO, LANG) + ' '*6 + '#\n' + '#'*24 + '\n' + Fore.RESET)

    # speech = rospy.ServiceProxy('speech_service', Talker)

    rospy.spin()