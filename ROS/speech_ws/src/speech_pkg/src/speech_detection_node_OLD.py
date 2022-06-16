#!/usr/bin/python3
import rospy
from speech_pkg.msg import SpeechData
from std_msgs.msg import Int16MultiArray, String

from demo_utils.io.audio import SpeechRecognitionVAD
from demo_utils.io.respeaker import ReSpeakerMicArrayV2
from demo_utils.ai.audio.voice_activity_detector.silero_vad import SileroVAD
from settings import demo_settings
from speech_recognition import AudioSource
from speech_pkg.srv import *
from utils import MySileroVad
# from pathlib import Path
import soundfile as sf
# import torch

import numpy as np
from time import sleep
from colorama import Fore

class ROSMicrophoneSource(AudioSource):

    def __init__(self, device_index=None, sample_rate=None, chunk_size=1024):
        self.device_index = device_index
        self.format = 8  # 16-bit int sampling
        self.SAMPLE_WIDTH = 2  # size in bytes of each sample (2 = 16 bit -> int16)
        self.SAMPLE_RATE = sample_rate  # sampling rate in Hertz
        self.CHUNK = chunk_size  # number of frames stored in each buffer

        self.audio = None
        self.stream = None


    def __enter__(self):
        self.stream = self.ROSAudioStream()

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        pass

    class ROSAudioStream:
    
        def read(self, chunk):
            audio = rospy.wait_for_message('mic_data', Int16MultiArray)
            return np.array(audio.data, dtype='int16').tobytes()

# Events topic publisher
event_pub = rospy.Publisher('events', String, queue_size=0)

class SpeechDetectionNode:
    '''SpeechDetectionNode implements the ROS interface for speech detection.

    The node subscribes to the following topics:

    - **mic_data** : Int16MultiArray.

    The node publishes on the following topics:

    - **speech_detection** : SpeechData custom message.

    The available methods are:
    
    - **\_\_init\_\_(self)**: constructor
    - **start(self)**: starts the ros node instance
    '''
    def events_mng(self,data):
        if data.data == demo_settings.io.speech.calibration_event:
            # Environment calibration
            rospy.loginfo(f"Calibration started")
            self.speechRecognition.calibrate()
            rospy.loginfo(f"Calibration done")
        elif data.data == 'ASR/TextListened':
            event_pub.publish("VAD/Disabled")
            self.enabled = False
        elif data.data == 'FieraMain/TextSpokenDone':
            event_pub.publish("VAD/Enabled")
            self.enabled = True
        # elif data.data == 'HandLeftBackTouched':
        #     self.reset()

    def reset(self):
        self.enabled = False
        event_pub.publish("VAD/Disabled")
        self.speechRecognition.calibrate()

    def start(self):
        print(Fore.CYAN + '#'*10 + ' Speech detector is running ' + '#'*10 + Fore.RESET)
        # Node and publisher initialization
        pub = rospy.Publisher('speech_detection', SpeechData, queue_size=3)
        rospy.init_node('speech_detection_node')

        # ReSpeaker interface initialization
        self.respeaker = ReSpeakerMicArrayV2()

        # Auxiliary VAD
        silero = MySileroVad(
            threshold=demo_settings.ai.audio.vad.threshold,
            sampling_rate=demo_settings.ai.audio.vad.sampling_rate
        )


        # VAD initialization        
        self.speechRecognition = SpeechRecognitionVAD(
            device_index = demo_settings.io.speech.device_index,
            sample_rate = demo_settings.io.speech.sample_rate,
            chunk_size = demo_settings.io.speech.chunk_size,
            timeout = 1,
            phrase_time_limit = demo_settings.io.speech.phrase_time_limit,
            calibration_duration = demo_settings.io.speech.calibration_duration,
            format = demo_settings.io.speech.format,
            source=ROSMicrophoneSource(
                demo_settings.io.speech.device_index,
                demo_settings.io.speech.sample_rate,
                demo_settings.io.speech.chunk_size
            ),
            vad=silero
        )
        print("vad loaded")
        # Events broker subscription
        rospy.Subscriber("events", String, self.events_mng)

        # Environment calibration
        self.speechRecognition.calibrate()


        # Loop
        print("before loop")
        i = 0
        self.enabled = True
        while not rospy.is_shutdown():
            if not self.enabled:
                sleep(.1)

                continue

            # Get speech data
            speech, timestamps = self.speechRecognition.get_speech_frame()

            if speech is None:
                continue

            # Disable
            self.enabled = False
            event_pub.publish("VAD/Disabled")

            # Message preparing
            msg = SpeechData()
            msg.data = speech.tolist()
            msg.doa = self.respeaker.get_doa() - 90
            msg.start_time = timestamps[0]
            msg.end_time = timestamps[1]

            # Message publishing
            manger_service(msg)
            # pub.publish(msg)
            speech_save = np.reshape(speech.copy(), (-1, 1))
            sf.write(f"/home/felice/felice/speech/code/detected_voices/{i}.wav", data=speech_save, samplerate=demo_settings.io.speech.sample_rate,format="WAV")
            i += 1
            self.enabled = True
            rospy.logdebug('Speech published with timestamps')

if __name__ == '__main__':
    rospy.loginfo("Waiting manager service")
    rospy.wait_for_service('manager_service')
    rospy.loginfo("manager service found")
    manger_service = rospy.ServiceProxy('manager_service', Manager)
    speech_detection = SpeechDetectionNode()
    speech_detection.start()
    