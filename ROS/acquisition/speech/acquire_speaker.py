import pyaudio as pa
import numpy as np
import soundfile as sf
import argparse
from threading import Thread
import time
from pathlib import Path
import os
import json
from commands import ADD_COMMANDS as COMMANDS
from colorama import Fore


'''
python3 acquire_speaker.py -sr 16000 -id_device 24
'''
LANGS = ["eng", "ita"]

def get_curr_dir():
    return os.path.abspath(os.path.dirname(os.path.relpath(__file__)))

class Microphone:

    def __init__(self):
        args = self.configure_parser()
        self.sr = args.sr
        self.CHANNELS = 6
        self.id_device = args.id_device
        self.seconds_to_acquire = args.rec_time
        if self.seconds_to_acquire is not None and int(self.seconds_to_acquire) == 0:
            raise Exception("Recording time cannot be zero")
        self.fname = args.fname
        self.frames_per_buffer = args.frames_per_buffer

        #self.stream = self.open_stream()

        dataset_path = Path(get_curr_dir()).joinpath("recordings_new")
        os.makedirs(dataset_path, exist_ok=True)
        dir_list = os.listdir(dataset_path)
        if dir_list == []:
            self.subject = 0
        else:
            self.subject = len(dir_list)
        self.subject_folder = os.path.join(dataset_path, '{0:06}'.format(self.subject))
        for lang in LANGS:
            os.makedirs(os.path.join(self.subject_folder, lang), exist_ok=True)

        with open("subject_info.json", "r") as f:
            self.info_dict = json.load(f)
        
        info_path = os.path.join(self.subject_folder, "info.txt")
        with open(info_path, "w") as f:
            json.dump(self.info_dict, f)

        #signal.signal(signal.SIGINT, self.break_loop)


    def configure_parser(self):
        args_parser = argparse.ArgumentParser()
        args_parser.add_argument("-sr", help="sampling rate", type=int, dest="sr", required=True)
        args_parser.add_argument("-id_device", help="input device index", dest="id_device", required=True, type=int)
        args_parser.add_argument("-recording_time",
                                 help="recording time in seconds, if you desire continuous recording ignore this parameter",
                                 default=None,
                                 dest="rec_time",
                                 required=False)
        args_parser.add_argument("-frames_per_buffer", default=1024, dest="frames_per_buffer",
                                 help="number of frames per read, default=1024", required=False)
        args_parser.add_argument("-fname", help="File name, do not specify the file extension. "
                                                "If this parameter is not specified "
                                                "the software generates the file name automatically",
                                 dest="fname", required=False, default=None)
        args = args_parser.parse_args()
        return args


    def convert_channel(self):
        for i, audio_array in enumerate(self.audio_array_list):
            audio_array = np.reshape(audio_array, (self.frames_per_buffer, self.CHANNELS))
            if i == 0:
                audio_channel = audio_array
                continue
            audio_channel = np.append(audio_channel, audio_array, axis=0)
        return audio_channel


    def open_stream(self):

        p = pa.PyAudio()

        stream = p.open(
            rate=self.sr,
            format=pa.paInt16,
            channels=self.CHANNELS,
            input=True,
            input_device_index=self.id_device,
            stream_callback=None
        )

        return stream

    @property
    def sampling_rate(self):
        return self.sr


    def save_audio(self, audio, lang, command):
        file_path = os.path.join(self.subject_folder, lang, "{}_{}.wav".format(lang, command))
        sf.write(file_path, audio, samplerate=self.sampling_rate, format="WAV")

    def thread_function(self):
        input()

    def acquire_audio(self):
        self.stream = self.open_stream()
        flag = True if self.seconds_to_acquire is None else False
        self.audio_array_list = []
        t = Thread(target=self.thread_function)
        t.start()
        while t.is_alive():
            audio_data = self.stream.read(self.frames_per_buffer)
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            self.audio_array_list.append(audio_array)
        time.sleep(0.3)
        self.stream.close()

if __name__ == "__main__":
    mic = Microphone()
    acquired_samples = {
        "eng":list(),
        "ita":list()
    }
    for lang in LANGS:
        for command, text in COMMANDS[lang].items():
            repeat = True
            audio = None
            while repeat:
                print("Say:"+Fore.GREEN+ "<{}>".format(text.upper())+Fore.RESET+"\t(<Enter> to stop recording)")
                mic.acquire_audio()
                audio = mic.convert_channel()
                in_str = input("Do you want repeat the command? (y/n)")
                repeat = False if in_str=='n' or in_str=='no' else True
            mic.save_audio(audio=audio, lang=lang, command=command)