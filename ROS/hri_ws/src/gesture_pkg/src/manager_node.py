#!/usr/bin/python3

import sys
import rospy
import argparse
from speech_pkg.srv import *
import uuid

from colorama import Fore
from datetime import datetime
from commands import GESTURE_COMMANDS
from speech_pkg.msg import Command, Gesture
from demo_utils.post_request import MyRequestPost


GESTURE_INFO_FILE = '/home/felice/command_interaction/acquisition/gesture/detected_voices/res.txt'

gesture_counter = 0
robot_listening = False
robot_uuid = uuid.uuid1().node


def publish_cmd(command:int, confidence:float):
    global robot_uuid

    # Make Command message
    cmd_msg = Command()
    cmd_msg.label = command
    cmd_msg.english = GESTURE_COMMANDS[command]["eng"]
    cmd_msg.italian = GESTURE_COMMANDS[command]["ita"]

    # Make Gesture message
    gesture_msg = Gesture()
    gesture_msg.id = 'UNISA.SpeechGestureAnalysis.Gesture:{}'.format(robot_uuid)
    gesture_msg.type = 'Gesture'
    gesture_msg.timestamp = datetime.now().isoformat()
    gesture_msg.command = cmd_msg
    gesture_msg.confidence = float(confidence)

    # print(Fore.LIGHTYELLOW_EX + '#'*30 + '\n' + str(gesture_msg) + '\n' + '#'*30 + Fore.RESET)

    rospy.loginfo(gesture_msg)
    pub.publish(gesture_msg)


def run_demo7(req):
    global GESTURE_INFO_FILE, gesture_counter, robot_listening, post_request, offset

    # CHANGE res ACCORDING THE SSD OUTPUT
    cmd = msg.header.frame_id
    confidence = msg.score
    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    # print(Fore.MAGENTA + '#'*10 + ' Detected command ' + '#'*10 + '\n{}\n'.format(res) + '#'*38 + Fore.RESET)

    if not robot_listening and res.cmd == 0:
        robot_listening = True
        print(Fore.LIGHTGREEN_EX + '-'*12 + ' ROBOT IS LISTENING ' + '-'*12 + Fore.RESET)

    if robot_listening:
        # publish_cmd(command=res.cmd + offset, confidence=res.probs[res.cmd])
        cmd = res.cmd + offset
        post_request.send_command(command_id=cmd, confidence=res.probs[res.cmd])
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", required=True, dest="demo", type=int)
    args, unknown = parser.parse_known_args(args=rospy.myargv(argv=sys.argv)[1:])

    detector = OneStageDetector(conf_thresh=0.4, size_threhsold=None)
    pub = rospy.Publisher('/UNISA/SpeechGestureAnalysis/Gesture', Gesture, queue_size=1)
    callback = Callback(pub, detector)
    sub = rospy.Subscriber("in_rgb", Image, callback)
    
    rospy.init_node('manager')
    
    rospy.wait_for_service('classifier_service')
    # rospy.wait_for_service('speech_service')

    command_eng = DEMO_CMD_ENG
    command_ita  = DEMO_CMD_ITA

    if args.demo == 3:
        offset = 0
        rospy.Service('manager_service', Manager, run_demo3)
    else:
        offset = 2
        rospy.Service('manager_service', Manager, run_demo7)
    # rospy.Service('manager_service', Manager, run_demo7)
    # rospy.Service('manager_service', Manager, lambda req: run(req, speech_counter))

    FIWARE_CB = rospy.get_param("/fiware_cb")
    post_request = MyRequestPost(robot_uuid, entity="UNISA.SpeechGestureAnalysis.Speech", msg_type="Speech", address=FIWARE_CB, port=1026)
    post_request.create_entity()
    
    classify = rospy.ServiceProxy('classifier_service', Classification)
    print(Fore.GREEN + '\n' + '#'*20 + '\n#   SYSTEM READY   #\n#' + ' '*6 + 'demo {}'.format(args.demo) + ' '*6 + '#\n' + '#'*20 + '\n' + Fore.RESET)

    # speech = rospy.ServiceProxy('speech_service', Talker)

    rospy.spin()