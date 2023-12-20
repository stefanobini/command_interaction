#!/usr/bin/python3

import rospy
from speech_pkg.srv import *
import uuid
from colorama import Fore
from datetime import datetime
import time

from std_msgs.msg import String
from commands import MAPPING, DEMO_FULL
from speech_pkg.msg import Command, Speech
from demo_utils.post_request import MyRequestPost


SPEECH_INFO_FILE = '/home/felice/command_interaction/ROS/detected_voices/res.txt'
speech_counter = 0
robot_uuid = uuid.uuid1().node
CMD_THRESHOLD = 0.0


# NOT USED METHOD
def publish_cmd(command:int, confidence:float):
    global command_eng, command_ita, robot_uuid

    # Make Command message
    cmd_msg = Command()
    cmd_msg.label = command
    cmd_msg.english = DEMO_FULL["eng"][command]
    cmd_msg.italian = DEMO_FULL["ita"][command]

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


def run_demo(req):
    global SPEECH_INFO_FILE, speech_counter, post_request, FIWARE_CB, DEMO
    res = classify(req.data)
    cmd = MAPPING[DEMO][res.cmd]
    prob = res.probs[res.cmd]
    print("Command: <{}>\tProbability: {:.3f}".format(DEMO_FULL["eng"][cmd], prob))
    #cb_reply_time = time.time()
    if cmd != len(DEMO_FULL["eng"])-1 and prob > CMD_THRESHOLD: # valid command is detected
        if FIWARE_CB == "None":
            pub.publish(DEMO_FULL["eng"][cmd])
            res_str = Fore.CYAN + '#'*6 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*6 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(DEMO_FULL["eng"][cmd], prob) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(DEMO_FULL["ita"][cmd], prob) + Fore.CYAN + ' #\n' + '#'* 44 + Fore.RESET + '\n'
            print(res_str)
        else:
            post_request.send_command(command_id=cmd, confidence=prob)
    #cb_reply_time = time.time() - cb_reply_time
    #print("COMUNICATION TIME: {:.4f} s".format(cb_reply_time))
    if rospy.get_param("/save_speech") == True:
        with open(SPEECH_INFO_FILE, "w") as f:
            res_str += '#'*6 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*6 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(DEMO_FULL["eng"][cmd], prob) + Fore.CYAN + ' #\n# ' + '{}: {:.3f}\n{}'.format(DEMO_FULL["ita"][cmd], prob, res.probs) + ' #\n' + '#'* 44 + '\n'
            f.write(res_str)
    speech_counter += 1
    return ManagerResponse(True)    # res.flag


if __name__ == "__main__":
    DEMO = rospy.get_param("/demo")
    LANG = rospy.get_param("/language")
    FIWARE_CB = rospy.get_param("/fiware_cb")

    pub = rospy.Publisher('speech_command', String, queue_size=1)
    rospy.init_node('manager')
    rospy.wait_for_service('classifier_service')
    # rospy.wait_for_service('speech_service')
    rospy.Service('manager_service', Manager, run_demo)

    if FIWARE_CB != "None":
        post_request = MyRequestPost(robot_uuid, entity="UNISA.SpeechGestureAnalysis.Speech", msg_type="Speech", address=FIWARE_CB, port=1026)
        post_request.create_entity()

    classify = rospy.ServiceProxy('classifier_service', Classification)
    print(Fore.GREEN + '\n' + '#'*24 + '\n#     SYSTEM READY     #\n#' + ' '*6 + 'demo {} {}'.format(DEMO, LANG) + ' '*6 + '#\n' + '#'*24 + '\n' + Fore.RESET)

    # speech = rospy.ServiceProxy('speech_service', Talker)

    rospy.spin()
