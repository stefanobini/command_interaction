#!/usr/bin/python
# coding=utf-8
import rospy
# import qi
from commands import command_eng, command_ita
from speech_pkg.srv import *
import argparse
from lang_settings import AVAILABLE_LANGS
import sys
import time
# from demo_utils.io.postRequest import MyRequestPost
from datetime import datetime
from speech_pkg.msg import Command, Speech

from colorama import Fore

ENG = {
    0: "Command predicted:",
    1: "Probability of rejection:",
    2: "Top 3 classes predicted:"
}

ITA = {
    0: "Comando predetto:",
    1: "Probabilità di rigetto:",
    2: "Top 3 classi predette:"
}

#CB_ADDRESS = '172.17.0.1'
#CB_PORT = 1026

speech_counter = 0

class TextController:
    def __init__(self, lang):
        self.lang = lang
        self.db_lang = ENG if self.lang == "eng" else ITA

    def get_lang_string(self, index):
        return self.db_lang[index]

def get_command_str(index):
    return commands_list[index]

def get_bests(probs):
    assert len(command_eng) == len(command_ita)
    cmds = list(range(len(probs)))
    values_dict = dict(zip(cmds, probs))
    #reject_key = len(command_eng)-1
    reject_prob = round(values_dict[len(command_eng)-1], 3)
    ## del values_dict[reject_key]
    values_list = list(values_dict.items())
    values_list.sort(key=lambda x: x[1], reverse=True)
    bests = values_list[:N_BEST_VALUES]
    bests = list(map(lambda x: (x[0], round(x[1], 3)), bests))
    return bests, reject_prob

def create_string(cmd, bests, reject_prob):
    out_str = ""
    out_str += Fore.GREEN + text_controller.get_lang_string(0) + " " + get_command_str(cmd) + Fore.RESET + '\n'
    out_str += Fore.YELLOW + text_controller.get_lang_string(1) + " " + str(reject_prob) + Fore.RESET + '\n'
    out_str += Fore.CYAN + text_controller.get_lang_string(2) + Fore.RESET + "\n"
    for cmd, prob in bests:
        out_str += Fore.CYAN + "%s %s\n" % (str(prob), get_command_str(cmd)) + Fore.RESET
    return out_str


def callback(req):
    global speech_counter
    bests, reject_prob = get_bests(req.probs)

    # Send message with FIROS
    #reqPost.send_command(bests[0][0], bests[0][1])  # id, confidence

    out_str = create_string(req.cmd, bests, reject_prob)
    with open("/home/felice/felice/speech/code/detected_voices/res.txt", "a") as fil:
        fil.write("*"*11 + ' {0:06d} '.format(speech_counter) + "*"*11)
        fil.write(out_str)
        fil.write("*" * 30)
        fil.write("\n")
    speech_counter += 1
    print(out_str)
    # say(get_command_str(req.cmd))
    # print(get_command_str(req.cmd))
    return TalkerResponse(True)

'''
def init_dict():
    command_eng[len(command_eng)] = "I do not understand"
    command_ita[len(command_ita)] = "Comando non supportato"
'''

def connect_robot():
    # Connect to the robot
    print("Connecting to robot...")
    session = qi.Session()
    session.connect('tcp://%s:9559' % IP )  # Robot IP
    print("Robot connected")

    motion_service = session.service("ALMotion")
    motion_service.wakeUp()

    #TextToSpeech service
    tts = session.service("ALTextToSpeech")
    tts.setLanguage("Italian" if args.lang == "ita" else "English")
    tts.say("Hello")
    return tts

def say(out_str):
    try:
        tts.say(out_str)
    except Exception:
        session = qi.Session()
        session.connect('tcp://%s:9559' % IP )

        tts = session.service("ALTextToSpeech")
        tts.setLanguage("Italian" if args.lang == "ita" else "English")
        tts.say(out_str)
    # time.sleep(0.5)

if __name__ == "__main__":
    N_BEST_VALUES = 3
    parser = argparse.ArgumentParser()
    parser.add_argument("--lang", required=True, dest="lang", type=str)
    parser.add_argument("--ip", required=True, dest="ip", type=str)
    args, unknown = parser.parse_known_args(args=rospy.myargv(argv=sys.argv)[1:])
    IP = args.ip
    if args.lang not in AVAILABLE_LANGS:
        raise Exception("Selected lang not available.\nAvailable langs:", AVAILABLE_LANGS)
    #init_dict()
    
    # tts = connect_robot()
    text_controller = TextController(args.lang)
    rospy.init_node('talker')
    commands_list = command_eng if args.lang == "eng" else command_ita

    # requestPost = MyRequestPost(CB_ADDRESS, CB_PORT)

    rospy.Service('speech_service', Talker, callback)   # cancelled the requestPost

    rospy.spin()