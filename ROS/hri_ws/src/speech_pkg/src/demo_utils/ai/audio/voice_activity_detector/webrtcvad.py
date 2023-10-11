from . import VoiceActivityDetector
import numpy as np
import webrtcvad

class GMM_VAD(VoiceActivityDetector):
    def __init__(self,
                    mode: int = 1,
                    sampling_rate: int = 16000
                ):
        
        self.sampling_rate = sampling_rate
        self.vad = webrtcvad.Vad(mode)

    def is_speech(self, buffer):
        return self.vad.is_speech(buffer,self.sampling_rate)