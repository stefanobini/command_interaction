#!/usr/bin/python

import sys

import rospy
import actionlib    # Brings in the SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # Brings in the .action file and messages used by the move base action
import dynamic_reconfigure.client

from speech_pkg.srv import *
import uuid
from colorama import Fore
from datetime import datetime
import time

from std_msgs.msg import String, Header
from speech_pkg.msg import Command, Speech
from iri_object_transportation_msgs.msg import explicit_information

from speech_pkg.srv import Classification, ClassificationResponse, ClassificationMSI
from commands import DEMO3_CMD_ENG, DEMO3_CMD_ITA, DEMO7_CMD_ENG, DEMO7_CMD_ITA, DEMO7P_CMD_ENG, DEMO7P_CMD_ITA, DEMO_CMD_ENG, DEMO_CMD_ITA
#from commands_unique_list import DEMO_CMD_ENG, DEMO_CMD_ITA
from intents import INTENTS, EXPLICIT_INTENTS, IMPLICIT_INTENTS, INTENTS_MSIEXP0, INTENTS_MSIEXP1, EXPLICIT_INTENTS_MSIEXP1, INTENT_TO_ACTION
from demo_utils.post_request import MyRequestPost


SPEECH_INFO_FILE = '/home/felice/command_interaction/ROS/detected_voices/res.txt'
speech_counter = 0
robot_listening = True
robot_uuid = uuid.uuid1().node
START_THRESHOLD = 0.03
res_str = ""
LEFT_IDs, AHEAD_IDs, RIGHT_IDs, STOP_IDs = [6,7], [2,3], [8,9], [10,11]


# NOT USED METHOD
def publish_cmd(command, confidence):
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
    global SPEECH_INFO_FILE, speech_counter, robot_listening, command_eng, command_ita, post_request, offset, FIWARE_CB, res_str

    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    #print(Fore.MAGENTA + '#'*10 + ' Detected command ' + '#'*10 + '\n{}\n{:.3f}/{}\n'.format(res.cmd, res.probs[res.cmd], res.probs) + '#'*38 + Fore.RESET)

    """
    if not robot_listening and (res.cmd == 0 or (res.probs[0]>START_THRESHOLD and res.cmd == len(command_eng))):
        robot_listening = True
        print(Fore.LIGHTGREEN_EX + '-'*12 + ' ROBOT IS LISTENING ' + '-'*12 + Fore.RESET)
    """

    if robot_listening:
        # publish_cmd(command=res.cmd + offset, confidence=res.probs[res.cmd])
        cmd = res.cmd + offset
        prob = None
        #cb_reply_time = time.time()
        if FIWARE_CB == "None":
            pub.publish(command_eng[res.cmd] + " - " + command_ita[res.cmd])
            res_str = Fore.CYAN + '#'*6 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*6 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_eng[res.cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}\n'.format(command_ita[res.cmd], res.probs[res.cmd]) + Fore.CYAN + ' #\n' + '#'* 44 + Fore.RESET + '\n'
            print(res_str)
        elif res.probs[0]>START_THRESHOLD and res.cmd == len(command_eng)-1 and False:
            cmd = offset
            prob = res.probs[0]
            post_request.send_command(command_id=cmd, confidence=prob)
        elif res.cmd == len(command_eng)-1:
            cmd = offset-1
            prob = res.probs[res.cmd]
        else: #res.cmd != len(command_eng)-1:
            cmd = res.cmd + offset
            prob = res.probs[res.cmd]
            post_request.send_command(command_id=cmd, confidence=prob)
        #cb_reply_time = time.time() - cb_reply_time
        #print("COMUNICATION TIME: {:.4f} s".format(cb_reply_time))
        """
        res_str = Fore.CYAN + '#'*6 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*6 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_eng[cmd], prob) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}\n{}'.format(command_ita[cmd], prob, res.probs) + Fore.CYAN + ' #\n' + '#'* 44 + Fore.RESET + '\n'
        #print(res_str)
        #"""
        
        if rospy.get_param("/save_speech") == True:
            with open(SPEECH_INFO_FILE, "w") as f:
                res_str += '#'*6 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*6 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(command_eng[cmd], prob) + Fore.CYAN + ' #\n# ' + '{}: {:.3f}\n{}'.format(command_ita[cmd], prob, res.probs) + ' #\n' + '#'* 44 + '\n'
                f.write(res_str)

    """
    if robot_listening and res.cmd == 1:
        robot_listening = False
        print(Fore.LIGHTRED_EX + '-'*12 + ' ROBOT IS NOT LISTENING ' + '-'*12 + Fore.RESET)
    """

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


def run_demo_MSIexp0(req):
    global SPEECH_INFO_FILE, speech_counter, FIWARE_CB, LANG, pub

    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    

    # REPLACE WITH A THRESHOLD
    if res.probs[res.cmd] < 0.0:
        print("@@@ LOW CONFIDENCE @@@")
        return ManagerResponse(True)    # res.flag
    
    
    #print(Fore.MAGENTA + '#'*10 + ' Detected intents ' + '#'*10 + '\n{}\n{:.3f}\n'.format(res.intent, res.int_probs[res.intent]) + '#'*38 + Fore.RESET)
    if res.cmd == len(INTENTS_MSIEXP0)-1:
        return ManagerResponse(True)    # res.flag

    message = explicit_information()
    message.header = Header()
    message.header.frame_id = INTENTS_MSIEXP0[res.cmd][LANG][0]
    message.header.stamp = rospy.Time.now()
    message.fsr_values = [1. for i in range(5)]
    message.sw_values = [False for i in range(5)]
    message.sw_values[res.cmd] = True
    #pub.publish(command_eng[cmd] + " - " + command_ita[cmd])
    pub.publish(message)
    
    res_str = Fore.CYAN + '#'*10 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*10 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(INTENTS_MSIEXP0[res.cmd][LANG][0], res.probs[res.cmd]) + Fore.CYAN + ' #\n' + '#'* 44 + '\n'
    print(res_str)
    
    '''
    if rospy.get_param("/save_speech") == True:
        with open(SPEECH_INFO_FILE, "a") as f:
            f.write(res_str)
    '''
    speech_counter += 1

    return ManagerResponse(True)    # res.flag


def run_demo_MSIexp1(req):
    global SPEECH_INFO_FILE, speech_counter, FIWARE_CB, LANG, actioner

    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    #print(Fore.MAGENTA + '#'*10 + ' Detected intents ' + '#'*10 + '\n{}\n{:.3f}\n'.format(res.intent, res.int_probs[res.intent]) + '#'*38 + Fore.RESET)
    if res.intent == len(INTENTS_MSIEXP1)-1:
        return ManagerResponse(True)    # res.flag
    
    x, y = 0, 0
    if res.intent == 8:    # lazy stop
        time.sleep(0.5) # wait 0.5 s
        actioner.cancel_goal()
    elif res.intent == 9:    # immediately stop
        actioner.cancel_goal()
    elif res.intent in [10, 11]:   # not handled
        return ManagerResponse(True)    # res.flag
    else:
        x = INTENT_TO_ACTION[res.intent]["x"]
        y = INTENT_TO_ACTION[res.intent]["y"]
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Move 'x' meters forward along the x axis of the "map" coordinate frame 
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = 1.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        # Set velocity according to the urgency
        setPlannerParameters(max_vel_x=INTENT_TO_ACTION[res.intent]["speed"])
        #""" To action TIAGO robot
        # Sends the goal to the action server.
        actioner.send_goal(goal)
        # Waits for the server to finish performing the action.
        wait = actioner.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
        # Result of executing the action
            return actioner.get_result()
        #"""
    
    #res_str = Fore.CYAN + '#'*10 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*10 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(INTENTS_MSIEXP1[res.intent]["text"][LANG], res.int_probs[res.intent]) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(EXPLICIT_INTENTS_MSIEXP1[LANG][res.explicit], res.exp_probs[res.explicit]) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(IMPLICIT_INTENTS[res.implicit][LANG], res.imp_probs[res.implicit]) + Fore.CYAN + ' #\n' + '#'* 44 + '\n'
    res_str = Fore.CYAN + '#'*10 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*10 + '\n# {}: {:.3f} #\n#  #\n'.format(INTENTS_MSIEXP1[res.intent]["text"][LANG], res.int_probs[res.intent]) + '#'* 44 + '\n'
    print(res_str)

    if rospy.get_param("/save_speech") == True:
        with open(SPEECH_INFO_FILE, "a") as f:
            f.write(res_str)

    speech_counter += 1
    # res = speech(res.cmd, res.probs)

    return ManagerResponse(True)    # res.flag


def run_demo_msi(req):
    global SPEECH_INFO_FILE, speech_counter, FIWARE_CB, LANG, pub

    # print(Fore.GREEN + '#'*22 + '\n# Manager is running #\n' + '#'*22 + Fore.RESET)
    res = classify(req.data)
    #print(Fore.MAGENTA + '#'*10 + ' Detected intents ' + '#'*10 + '\n{}\n{:.3f}\n'.format(res.intent, res.int_probs[res.intent]) + '#'*38 + Fore.RESET)
    if res.intent == -1:
        print(Fore.YELLOW + "Not in the set")
        return ManagerResponse(True)    # res.flag
    
    intent = res.intent
    if intent in LEFT_IDs:
        intent = 0
    elif intent in AHEAD_IDs:
        intent = 1
    elif intent in RIGHT_IDs:
        intent = 2
    elif intent in STOP_IDs:
        intent = 3
    else:
        intent = -1
    
    if intent == -1:
        print(Fore.YELLOW + "Not a move intent")
        return ManagerResponse(True)    # res.flag
    
    message = explicit_information()
    message.header = Header()
    message.header.frame_id = INTENTS[res.intent]["text"][LANG]
    message.header.stamp = rospy.Time.now()
    message.fsr_values = [1. for i in range(5)]
    message.sw_values = [False for i in range(5)]
    message.sw_values[intent] = True
    #pub.publish(command_eng[cmd] + " - " + command_ita[cmd])
    pub.publish(message)
    
    res_str = Fore.CYAN + '#'*10 + ' SPEECH CHUNCK n.{0:06d} '.format(speech_counter) + '#'*10 + '\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(INTENTS[res.intent]["text"][LANG], res.int_probs[res.intent]) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(EXPLICIT_INTENTS[LANG][res.explicit], res.exp_probs[res.explicit]) + Fore.CYAN + ' #\n# ' + Fore.LIGHTCYAN_EX + '{}: {:.3f}'.format(IMPLICIT_INTENTS[res.implicit][LANG], res.imp_probs[res.implicit]) + Fore.CYAN + ' #\n' + '#'* 44 + '\n'
    print(res_str)
    
    if rospy.get_param("/save_speech") == True:
        with open(SPEECH_INFO_FILE, "a") as f:
            f.write(res_str)

    speech_counter += 1
    # res = speech(res.cmd, res.probs)

    return ManagerResponse(True)    # res.flag


def setPlannerParameters(max_vel_x):
    node_to_reconfigure = "/move_base/PalLocalPlanner"
    client = dynamic_reconfigure.client.Client(node_to_reconfigure)
    params = {
        #'acc_lim_x': 0.65,
        'max_vel_x': max_vel_x
        }
    client.update_configuration(params)
    return True

if __name__ == "__main__":
    DEMO = rospy.get_param("/demo")
    LANG = rospy.get_param("/language")
    FIWARE_CB = rospy.get_param("/fiware_cb")
    classify = None

    pub = rospy.Publisher('speech_command', String, queue_size=1)
    # pub = rospy.Publisher('/UNISA/SpeechGestureAnalysisAWS/Speech', Speech, queue_size=10)
    # pub = rospy.Publisher('/UNISA/SpeechGestureAnalysisCOBOT/Speech', Speech, queue_size=10)
    actioner = None
    
    rospy.init_node('manager')
    
    rospy.wait_for_service('classifier_service')
    # rospy.wait_for_service('speech_service')

    if DEMO == str(3):
        offset = 0
        command_eng = DEMO3_CMD_ENG
        command_ita = DEMO3_CMD_ITA
        rospy.Service('manager_service', Manager, run_demo3)
        classify = rospy.ServiceProxy('classifier_service', Classification)
    elif DEMO == str(7):
        offset = 5
        command_eng = DEMO7_CMD_ENG
        command_ita = DEMO7_CMD_ITA
        rospy.Service('manager_service', Manager, run_demo7)
        classify = rospy.ServiceProxy('classifier_service', Classification)
    elif DEMO == "7_plus":
        offset = 5
        command_eng = DEMO7P_CMD_ENG
        command_ita = DEMO7P_CMD_ITA
        rospy.Service('manager_service', Manager, run_demo7)
        classify = rospy.ServiceProxy('classifier_service', Classification)
    elif DEMO == "full":
        offset = 0
        command_eng = DEMO_CMD_ENG
        command_ita = DEMO_CMD_ITA
        rospy.Service('manager_service', Manager, run_demo_full)
        classify = rospy.ServiceProxy('classifier_service', Classification)
    elif DEMO == "msi":
        rospy.Service('manager_service', Manager, run_demo_msi)
        classify = rospy.ServiceProxy('classifier_service', ClassificationMSI)
        pub = rospy.Publisher('feedback_data', explicit_information, queue_size=1)
    elif DEMO == "MSIexp0":
        rospy.Service('manager_service', Manager, run_demo_MSIexp0)
        classify = rospy.ServiceProxy('classifier_service', Classification)
        pub = rospy.Publisher('feedback_data', explicit_information, queue_size=1)
    elif DEMO == "MSIexp1":
        rospy.Service('manager_service', Manager, run_demo_MSIexp1)
        classify = rospy.ServiceProxy('classifier_service', ClassificationMSI)
        actioner = actionlib.SimpleActionClient('move_base',MoveBaseAction) # Create an action client called "move_base" with action definition file "MoveBaseAction"
        actioner.wait_for_server()  # Waits until the action server has started up and started listening for goals.
        #tiago_service = rospy.ServiceProxy('move_base/PalLocalPlanner/set_parameters', Classification)
        
    #command_eng = DEMO_CMD_ENG
    #command_ita = DEMO_CMD_ITA
    # rospy.Service('manager_service', Manager, run_demo7)
    # rospy.Service('manager_service', Manager, lambda req: run(req, speech_counter))

    if FIWARE_CB != "None":
        post_request = MyRequestPost(robot_uuid, entity="UNISA.SpeechGestureAnalysis.Speech", msg_type="Speech", address=FIWARE_CB, port=1026)
        post_request.create_entity()

    print(Fore.GREEN + '\n' + '#'*24 + '\n#     SYSTEM READY     #\n#' + ' '*6 + 'demo {} {}'.format(DEMO, LANG) + ' '*6 + '#\n' + '#'*24 + '\n' + Fore.RESET)

    # speech = rospy.ServiceProxy('speech_service', Talker)

    rospy.spin()