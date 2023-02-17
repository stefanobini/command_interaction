#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray
import speech_recognition as sr
from speech_pkg.msg import SystemHealth
from datetime import datetime
import uuid

from demo_utils.post_alive import MyRequestPost
from demo_utils.io.audio import PyAudioSource
from settings import demo_settings


class MicrophoneNode:
    '''MicrophoneNode implements the ROS interface for the microphone acquisition.

    The node has not subscription to any topic.

    The node publishes on the following topics:

    - **mic_data** : Int16MultiArray

    The available methods are:
    
    - **\_\_init\_\_(self)**: constructor
    - **start(self)**: starts the ros node instance
    '''

    def start(self):
        # Node and publisher initialization
        pub = rospy.Publisher('mic_data', Int16MultiArray, queue_size=3)
        # health_pub = rospy.Publisher('/UNISA/SpeechGestureAnalysis/SystemHealth', SystemHealth, queue_size=10)
        
        rospy.init_node('microphone_node')

        # Stream initialization
        audio_stream = PyAudioSource(
            device_index = demo_settings.io.mic.device_index,
            sample_rate = demo_settings.io.mic.sample_rate,
            channels = demo_settings.io.mic.channels,
            frames_per_buffer = demo_settings.io.mic.frames_per_buffer,
            format = demo_settings.io.mic.format
        )

        '''
        robot_uuid = uuid.uuid1()

        # Make SystemHealth message
        alive_msg = SystemHealth()
        alive_msg.id = 'UNISA.SpeechGestureAnalysis.SystemHealth:{}'.format(robot_uuid)
        alive_msg.type = 'SystemHealth'
        alive_msg.timestamp = datetime.now().isoformat()
        alive_msg.status = "Ok"
        '''

        prev_time = datetime.now()

        while not rospy.is_shutdown():
            # Get data
            audio_frame = audio_stream.get_audio_frame()
            
            # Message Audio message
            msg = Int16MultiArray()
            msg.data = audio_frame
            pub.publish(msg)

            curr_time = datetime.now()
            if (curr_time - prev_time).total_seconds() > 60:
                prev_time = curr_time
                '''
                alive_msg.timestamp = datetime.now().isoformat()
                alive_msg.status = "Ok"
                health_pub.publish(alive_msg)
                '''
                if FIWARE_CB != "None":
                    post_request.send_alive(status="Ok") # Comment to unable health message

        # alive_msg.status = "Not alive"
        if FIWARE_CB != "None":
            post_request.send_alive(status="Error") # Comment to unable health message

        # Close the stream
        audio_stream.stop() 

if __name__ == '__main__':
    robot_uuid = uuid.uuid1().node
    FIWARE_CB = rospy.get_param("/fiware_cb")

    # Comment to unable health message
    if FIWARE_CB != "None":
        post_request = MyRequestPost(robot_uuid, entity="UNISA.SpeechGestureAnalysis.SystemHealth", msg_type="SystemHealth", address=FIWARE_CB, port=1026)
        post_request.create_entity()

    microphone = MicrophoneNode()
    microphone.start()