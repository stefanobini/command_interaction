import pyaudio as pa
import numpy as np
import soundfile as sf
import argparse
import signal
from threading import Timer
import time
import datetime
from pathlib import Path
import os
import json

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
        self.exit_flag = True
        self.frames_per_buffer = args.frames_per_buffer
        self.stream = self.open_stream()

        dataset_path = Path(get_curr_dir()).joinpath("audio_saves")
        dataset_path.mkdir(parents=True, exist_ok=True)
        dir_list = os.listdir(dataset_path)
        if dir_list == []:
            self.subject = 0
        else:
            self.subject = len(dir_list)
        self.subject_folder = os.path.join(dataset_path, '{0:06}'.format(self.subject))
        os.mkdir(self.subject_folder)

        with open("subject_info.json", "r") as f:
            self.info_dict = json.load(f)
        
        info_path = os.path.join(self.subject_folder, "info.txt")
        with open(info_path, "w") as f:
            json.dump(self.info_dict, f)

        signal.signal(signal.SIGINT, self.break_loop)


    def loop(self):
        flag = True if self.seconds_to_acquire is None else False
        self.audio_array_list = []
        if flag:
            print("Ctrl-C to stop recording")
        else:
            print("Recording for", int(self.seconds_to_acquire))
            timer = Timer(int(self.seconds_to_acquire), self.break_loop)
            timer.start()
        while self.exit_flag:
            audio_data = self.stream.read(self.frames_per_buffer)
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            self.audio_array_list.append(audio_array)


    def break_loop(self, signum=0, frame=0):
        self.exit_flag = False
        time.sleep(0.3)
        self.stream.close()


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

    def save_audio(self, audio):
        if self.fname is not None:
            fname = self.fname
        else:
            fname = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S.%f")[:-3]
        fname += ".wav"
        path = os.path.join(self.subject_folder, fname)
        if os.path.isdir(path):
            name_split = fname.split(".")[0]
            name = name_split.split(fname)[0]
            i = 1
            while path.exists():
                fname = f"{name}_{i}.wav"
                path = os.path.join(self.subject_folder, fname)
                i += 1
            print(f"{self.fname}.wav already exists, created a new file: {fname}")
        sf.write(path, audio, samplerate=self.sampling_rate, format="WAV")

if __name__ == "__main__":
    mic = Microphone()
    mic.loop()
    audio = mic.convert_channel()
    mic.save_audio(audio)