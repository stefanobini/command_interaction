#!/usr/bin/python3

import rospy
from speech_pkg.msg import SpeechData
from std_msgs.msg import Int16MultiArray, String

from demo_utils.io.audio import SpeechRecognitionVAD
from demo_utils.io.respeaker import ReSpeakerMicArrayV2
from settings import demo_settings
from speech_recognition import AudioSource
# from scipy.io.wavfile import write
import numpy as np
from time import sleep
import soundfile as sf
from queue import Queue

from speech_pkg.srv import *    # added by BEIS
from time import sleep          # added by BEIS
from colorama import Fore       # added by BEIS


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
        self.stream.clean()


    class ROSAudioStream:
        def __init__(self):
            self.buffer = None
            self.subscriber = rospy.Subscriber('mic_data', Int16MultiArray, self.store)


        def store(self, audio):
            if self.buffer is not None:
                self.buffer.put(np.array(audio.data, dtype='int16').tobytes())


        def read(self, chunk):
            self.buffer = Queue() if self.buffer is None else self.buffer
            
            return self.buffer.get()
        
        
        def clean(self):
            self.subscriber.unregister()
            del self.buffer


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
            #self.speechRecognition.calibrate() # used in DeSimone demo
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
        #self.speechRecognition.calibrate() # used in DeSimone demo

    def start(self):
        # Node and publisher initialization
        # pub = rospy.Publisher('speech_detection', SpeechData, queue_size=3)   # commented by BEIS 
        rospy.init_node('speech_detection_node')

        # ReSpeaker interface initialization
        self.respeaker = ReSpeakerMicArrayV2()
        if (rospy.get_param("/vad") == "silero"):
            # Auxiliary VAD
            from demo_utils.ai.audio.voice_activity_detector.silero_vad import SileroVAD
            vad_model = SileroVAD(
                demo_settings.ai.audio.vad.model,
                demo_settings.ai.audio.vad.threshold,
                demo_settings.ai.audio.vad.sampling_rate, 
                demo_settings.ai.audio.vad.device
            )
        elif (rospy.get_param("/vad") == "webrtcvad"):
            from demo_utils.ai.audio.voice_activity_detector.webrtcvad import GMM_VAD
            vad_model = GMM_VAD(
                demo_settings.ai.audio.vad.mode,
                demo_settings.ai.audio.vad.sampling_rate
            )
        else:
            vad_model = None
            

        # VAD initialization        
        self.speechRecognition = SpeechRecognitionVAD(
            device_index = demo_settings.io.speech.device_index,
            sample_rate = demo_settings.io.speech.sample_rate,
            chunk_size = demo_settings.io.speech.chunk_size,
            timeout = 5,    # in the DeSimone demo is 1
            phrase_time_limit = demo_settings.io.speech.phrase_time_limit,
            calibration_duration = demo_settings.io.speech.calibration_duration,
            format = demo_settings.io.speech.format,
            source=ROSMicrophoneSource(
                demo_settings.io.speech.device_index,
                demo_settings.io.speech.sample_rate,
                demo_settings.io.speech.chunk_size
            ),
            vad = vad_model
        )

        # print(Fore.GREEN + '#'*14 + '\n# VAD loaded #\n' + '#'*14 + Fore.RESET)

        # Events broker subscription
        rospy.Subscriber("events", String, self.events_mng)

        # Environment calibration
        self.speechRecognition.calibrate()                  # there is not in the DeSimone demo
        self.speechRecognition.get_speech_frame(timeout=2)  # there is no timeout in the DeSimone demo


        # Loop
        i = 0
        self.enabled = True # False in Pepper demo
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
            # msg.header.stamp = rospy.Time.now()     # there is not in DeSimone demo
            msg.data = speech.tolist()
            msg.doa = self.respeaker.get_doa() - 90
            msg.start_time = timestamps[0]
            msg.end_time = timestamps[1]

            # Message publishing
            # pub.publish(msg)  # commented by BEIS
            manger_service(msg)

            if rospy.get_param("/save_speech") == True:
                speech_save = np.reshape(speech.copy(), (-1, 1))
                sf.write("{}/{}.wav".format(SPEECH_SAMPLES_PATH, i), data=speech_save, samplerate=demo_settings.io.speech.sample_rate, format="WAV")
                i += 1

            self.enabled = True

            rospy.logdebug('Speech published with timestamps')

if __name__ == '__main__':
    SPEECH_SAMPLES_PATH = '/home/felice/command_interaction/ROS/detected_voices'

    rospy.loginfo("Waiting manager service")                            # added by BEIS
    rospy.wait_for_service('manager_service')                           # added by BEIS
    rospy.loginfo("manager service found")                              # added by BEIS
    
    manger_service = rospy.ServiceProxy('manager_service', Manager)     # added by BEIS

    speech_detection = SpeechDetectionNode()
    speech_detection.start()