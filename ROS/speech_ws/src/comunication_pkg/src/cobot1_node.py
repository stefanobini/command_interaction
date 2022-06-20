#!/usr/bin/env python
# coding=utf-8
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

# import cmd
import rospy
# from rospy import Time
from datetime import datetime
# from std_msgs.msg import String
from comunication_pkg.msg import Command, Gesture, Speech

""" entities = {
    0:{'type':'Speech', 'entity':'UNISA.SpeechGestureAnalysis.Speech'},
    1:{'type':'Gesture', 'entity':'UNISA.SpeechGestureAnalysis.Gesture'}
} """
commands = {
    7: {'label':7, 'english':'Bring me the gun screwdriver', 'italian':"Portami l'avvitatore elettrico"},
    9: {'label':9, 'english':'Bring me the elbow screwdriver', 'italian':"Portami l'avvitatore a gomito"},
    13: {'label':13, 'english':'Bring me the screwdriver', 'italian':'Portami il cacciavite'},
    17: {'label':17, 'english':'Bring me the control panel', 'italian':'Portami la mostrina comandi'},
    19: {'label':19, 'english':'Bring me the rearviewer mirror', 'italian':'Portami lo specchietto retrovisore'},
    21: {'label':21, 'english':'Release', 'italian':'Rilascia'},
    24: {'label':24, 'english':'Start', 'italian':'Start'}
}


def publisher():
    pub = rospy.Publisher('/UNISA/SpeechGestureAnalysis/Speech', Speech, queue_size=10)
    rospy.init_node('cobot1_node')

    isExit = False

    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    while not isExit:
        command = int(input("______________________________________________\n" \
                            "| Select command:                            |\n" \
                            "| 24   -   Start                             |\n" \
                            "| 13   -   Bring me the screwdriver          |\n" \
                            "| 7    -   Bring me the gun screwdriver      |\n" \
                            "| 9    -   Bring me the elbow screwdriver    |\n" \
                            "| 17   -   Bring me the control panel        |\n" \
                            "| 19   -   Bring me the rearviewer mirror    |\n" \
                            "| 21   -   Release                           |\n" \
                            "______________________________________________\n"))

        if command in [24, 13, 7, 9, 17, 19, 21]:
            confidence = input("Confidence: ")
            
            # Make Command message
            cmd_msg = Command()
            cmd_msg.label = commands[command]['label']
            cmd_msg.english = commands[command]['english']
            cmd_msg.italian = commands[command]['italian']

            # Make Speech message
            speech_msg = Speech()
            # speech_msg.id = '/cobot1/speech'
            speech_msg.id = 'UNISA.SpeechGestureAnalysis.Speech'
            speech_msg.type = 'Speech'
            # speech_msg.timestamp = Time.now()
            speech_msg.timestamp = datetime.now().isoformat()
            speech_msg.command = cmd_msg
            speech_msg.confidence = float(confidence)

            print(speech_msg)

            #rospy.loginfo(speech_msg)
            pub.publish(speech_msg)

            isExit = False  # continue
        
        else:
            print("Bye Bye!!!")
            isExit = True  # exit
        

        # rate.sleep()


if __name__ == '__main__':

    try:
        publisher()
    except rospy.ROSInterruptException:
        pass