#!/usr/bin/python3

import os
#from model import Model
import numpy as np
#from nemo.core.neural_types import NeuralType, AudioSignal, LengthsType
#from nemo.core.classes import IterableDataset
from torch.utils.data import DataLoader
from speech_pkg.srv import Classification, ClassificationResponse
from settings import pepper, global_utils
import torch
import rospy
import sys
from pathlib import Path
import argparse
from lang_settings import AVAILABLE_LANGS
import colorama
colorama.init(autoreset=True)
from colorama import Fore, Back
import librosa
from PIL import Image

from commands import DEMO3_CMD_ENG, DEMO3_CMD_ITA, DEMO7_CMD_ENG, DEMO7_CMD_ITA, DEMO7P_CMD_ENG, DEMO7P_CMD_ITA, DEMO_CMD_ENG, DEMO_CMD_ITA
#from commands_unique_list import DEMO_CMD_ITA, DEMO_CMD_ENG
import time
import torchaudio
from dotmap import DotMap

#from mtl_exp.MTL_conf import settings
from settings.felice import settings
from mtl_exp.resnet8 import ResNet8_PL
from mtl_exp.hardsharing import HardSharing_PL
from mtl_exp.softsharing import SoftSharing_PL


MODEL_PATH = "models"


def get_melspectrogram(waveform:torch.Tensor) -> torch.Tensor:
        """Compute Mel-Spectrogram of a waveform given as input (waveform).
        The method use torchaudio library for the trasformation. All the parameters (sample_rate, n_fft, win_length, hop_length, n_mels) are taken from the configuration file.
        
        Parameters
        ----------
        waveform: torch.Tensor
            Waveform of an audio
        Returns
        -------
        torch.Tensor
            Mel spectrogram of the waveform
        """
        '''
        melspectrogram = librosa.feature.melspectrogram(y=waveform, sr=settings.input.sample_rate, S=None, n_fft=settings.input.spectrogram.n_fft, hop_length=settings.input.spectrogram.hop_length, win_length=settings.input.spectrogram.win_length, n_mels=settings.input.mel.n_mels, window='hann', center=True, pad_mode='constant', power=2.0)
        return np.float32(melspectrogram)
        '''
        transformation = torchaudio.transforms.MelSpectrogram(sample_rate=settings.input.sample_rate, n_fft=settings.input.spectrogram.n_fft, win_length=settings.input.spectrogram.win_length, hop_length=settings.input.spectrogram.hop_length, n_mels=settings.input.mel.n_mels).cuda()
        return transformation(waveform)

def amplitude_to_db_spectrogram(spectrogram:torch.Tensor) -> torch.Tensor:
        """Convert amplitude spectrogram in dB spectrogram.
        The method use torchaudio library.
        
        Parameters
        ----------
        spectrogram: torch.Tensor
            Spectrogram of the audio signal (Mel or MFCC)

        Returns
        -------
        torch.Tensor
            dB version of the input spectrogram
        """
        #### CHANGED HERE THE MULTIPLIER FROM 10. T0 20.
        spectrogram_db = torchaudio.functional.amplitude_to_DB(x=spectrogram, multiplier=20., amin=1e-10, db_multiplier=np.log10(max(spectrogram.cpu().max(), 1e-10)), top_db=80)
        #spectrogram_db = librosa.power_to_db(spectrogram, ref=1., amin=1e-10, top_db=80.0) # ref: 20 * log10(S / ref)
        #return  np.float32(spectrogram_db.cpu().numpy())
        return spectrogram_db

def preprocess(waveform: np.ndarray):
    #waveform = np.mean(waveform, axis=0, keepdims=True)
    #waveform = (waveform-np.min(waveform))/(np.max(waveform)-np.min(waveform))    # normalization
    waveform = np.expand_dims(waveform, axis=0) # add channel dimension
    waveform = torch.tensor(waveform, dtype=torch.float32).cuda()
    melspectrogram = get_melspectrogram(waveform=waveform)
    #print(Back.YELLOW + "SPECT INFO:\ntype:{}\tshape:{}\tdtype:{}\tmin:{}\tmax:{}\tmean:{}\n".format(type(melspectrogram), melspectrogram.shape, melspectrogram.dtype, torch.min(melspectrogram), torch.max(melspectrogram), torch.mean(melspectrogram)))
    db_melspectrogram = amplitude_to_db_spectrogram(spectrogram=melspectrogram)
    #return torch.tensor(db_melspectrogram, dtype=torch.float32).cuda()
    return db_melspectrogram


def select_parameters(language="eng", demo="7"):
    #models_path = Path(global_utils.get_curr_dir(__file__)).parent.joinpath("experiments")
    models_path = Path(global_utils.get_curr_dir(__file__)).parent.joinpath(MODEL_PATH)
    COMMANDS = None
    if demo == "3":
        if language == 'eng':
            COMMANDS = DEMO3_CMD_ENG
            #ckpt_folder = models_path.joinpath('eng', 'demo3_eng')
            #ckpt_name = 'matchcboxnet--val_loss=2.2774-epoch=209.model' # demo3_eng (old one)
        elif language == 'ita':
            COMMANDS = DEMO3_CMD_ITA
            #ckpt_folder = models_path.joinpath('ita', 'demo3_ita')
            #ckpt_name = 'matchcboxnet--val_loss=8.5081-epoch=184.model'   # demo3_ita
        ckpt_folder = models_path.joinpath('demo'+demo, language)
        ckpt_folder = models_path.joinpath(ckpt_folder, os.listdir(path=ckpt_folder)[-1], "checkpoints")
        #ckpt_folder = models_path.joinpath('demo'+demo, language, settings.logger.version, 'checkpoints')
        ckpt_name = os.listdir(path=ckpt_folder)[-1]
    elif demo == "7":
        if language == 'eng':
            COMMANDS = DEMO7_CMD_ENG
            #ckpt_folder = models_path.joinpath('eng', 'demo7_phase_I_eng_no_pre')
            #ckpt_name = "matchcboxnet--val_loss=1.4875-epoch=127.model" # demo7_phase_I_eng_no_pre
        elif language == 'ita':
            COMMANDS = DEMO7_CMD_ITA
            #ckpt_folder = models_path.joinpath("ita", 'demo7_phase_I_ita_no_pre')
            #ckpt_name = "matchcboxnet--val_loss=2.9228-epoch=123.model" # demo7_phase_I_ita_no_pre
        ckpt_folder = models_path.joinpath('demo'+demo, language)
        ckpt_folder = models_path.joinpath(ckpt_folder, os.listdir(path=ckpt_folder)[-1], "checkpoints")
        ckpt_name = os.listdir(path=ckpt_folder)[-1] 
    elif demo == "7_plus":
        if language == 'eng':
            COMMANDS = DEMO7P_CMD_ENG
            #ckpt_folder = models_path.joinpath('eng', 'demo7_phase_I_eng_no_pre')
            #ckpt_name = "matchcboxnet--val_loss=1.4875-epoch=127.model" # demo7_phase_I_eng_no_pre
        elif language == 'ita':
            COMMANDS = DEMO7P_CMD_ITA
            #ckpt_folder = models_path.joinpath("ita", 'demo7_phase_I_ita_no_pre')
            #ckpt_name = "matchcboxnet--val_loss=2.9228-epoch=123.model" # demo7_phase_I_ita_no_pre
        ckpt_folder = models_path.joinpath('demo'+demo, language)
        ckpt_folder = models_path.joinpath(ckpt_folder, os.listdir(path=ckpt_folder)[-1], "checkpoints")
        ckpt_name = os.listdir(path=ckpt_folder)[-1] 
    elif demo == "full":
        if language == 'eng':
            COMMANDS = DEMO_CMD_ENG
            # ckpt_folder = models_path.joinpath('eng', 'full_eng')
            ckpt_folder = models_path.joinpath('eng', 'new_full_eng')
            # ckpt_name = "matchcboxnet--val_loss=3.9556-epoch=249.model" # full_eng
            ckpt_name = "matchcboxnet--val_loss=3.5196-epoch=99.model" # new_full_eng
        elif language == 'ita':
            COMMANDS = DEMO_CMD_ITA
            ckpt_folder = models_path.joinpath("ita", 'full_ita')
            # ckpt_folder = models_path.joinpath("ita", 'new_full_ita')            

            ckpt_name = "matchcboxnet--val_loss=2.377-epoch=104.model"  # full_ita
            # ckpt_name = "matchcboxnet--val_loss=3.269-epoch=97.model"   # new_full_ita, here fix the commands
            # ckpt_name = "matchcboxnet--val_loss=2.5771-epoch=47.model"   # new_full_ita, here fix the commands

        ckpt_folder = models_path.joinpath('demo'+demo, language)
        ckpt_folder = models_path.joinpath(ckpt_folder, os.listdir(path=ckpt_folder)[-1], "checkpoints")
        ckpt_name = os.listdir(path=ckpt_folder)[-1]
    return COMMANDS, ckpt_folder, ckpt_name


def infer_signal(model, signal):
    data_layer.set_signal(signal)
    batch = next(iter(data_loader))
    audio_signal, audio_signal_len = batch
    audio_signal, audio_signal_len = audio_signal.to(model.device), audio_signal_len.to(model.device)
    logits = model.forward(input_signal=audio_signal, input_signal_length=audio_signal_len)
    return logits


class NewClassifier:
    def __init__(self, threshold_1, threshold_2, commands, ckpt_folder, ckpt_name):
        self.commands = commands
        self.threshold_1 = threshold_1
        self.threshold_2 = threshold_2

        labels = [i for i in range(len(commands))]

        assert settings.model.network in ["resnet8"]
        ckpt_path = os.path.join(ckpt_folder, ckpt_name)
        print(Back.YELLOW + ckpt_path)
        self.model = ResNet8_PL(settings=settings, num_labels=len(labels), loss_weights=None).cuda()  # Load model
        ckpt = torch.load(ckpt_path, lambda s, l: s)
        state_dict = ckpt["state_dict"]
        del state_dict["loss_fn.weight"]
        self.model.load_state_dict(state_dict=state_dict)
        self.model.eval()
    
        # In order to avoid the waste of time related to the first inference
        signal = np.zeros(shape=(32000))
        x = preprocess(waveform=signal)
        x = torch.unsqueeze(input=x, dim=0)   # add batch dimension
        self.model.predict(x)
        #self.model.predict(melspectrogram)

        self.init_node()


    def _pcm2float(self, sound: np.ndarray):
        abs_max = np.abs(sound).max()
        sound = sound.astype('float32')
        if abs_max > 0:
            sound *= 1 / abs_max
        sound = sound.squeeze()  # depends on the use case
        return sound

    def _numpy2tensor(self, signal: np.ndarray):
        signal_size = signal.size
        signal_torch = torch.as_tensor(signal, dtype=torch.float32)
        signal_size_torch = torch.as_tensor(signal_size, dtype=torch.int64)
        return signal_torch, signal_size_torch

    def convert(self, signal):
        signal = np.array(signal)
        signal_nw = self._pcm2float(signal)
        return signal_nw

    def predict(self, signal: np.ndarray):
        ###print(Back.BLUE + "SIGNAL INFO:\ntype:{}\tshape:{}\tdtype:{}\tmin:{}\tmax:{}\tmean:{}\n".format(type(signal), signal.shape, signal.dtype, np.min(signal), np.max(signal), np.mean(signal)))
        #signal, sample_rate = torchaudio.load("/home/felice/command_interaction/ROS/hri_ws/ita_0.wav")
        x = preprocess(waveform=signal)
        #'''
        #x_cpu = np.uint8(x[0].cpu().numpy())
        #x_cpu = (x_cpu-np.min(x_cpu))/(np.max(x_cpu)-np.min(x_cpu))
        #spect = Image.fromarray(x_cpu)
        #spect.save("/home/felice/command_interaction/ROS/hri_ws/spect_example.png")
        #'''
        x = torch.unsqueeze(input=x, dim=0)   # add batch dimension
        ###print(Back.GREEN + "INPUT INFO:\ntype:{}\tshape:{}\tdtype:{}\tmin:{}\tmax:{}\tmean:{}\n".format(type(x), x.shape, x.dtype, np.min(x_cpu), np.max(x_cpu), np.mean(x_cpu)))
        #plot_melspectrogram(path="mel_spec_db.png", melspectrogram=db_melspectrogram)
        #prev_time = time.time()
        #cmd_probs = self.cmd_model.predict(melspectrogram)
        #spk_probs = self.spk_model.predict(melspectrogram)
        cmd_probs = self.model.predict(x)
        #infer_time = time.time() - prev_time
        #print('INFER TIME: {:.4f}'.format(infer_time))
        cmd_probs = cmd_probs.cpu().detach().numpy()
        cmd = np.argmax(cmd_probs, axis=1)
        #print(cmd, cmd_probs[0])
        """
        if len(cmd_probs[0]) < len(self.commands):
            cmd_probs = np.append(arr=cmd_probs, values=[1-cmd_probs[0][cmd]], axis=1)
        """
        '''
        REJECT_LABEL = probs.shape[1] - 1
        # cmd = np.argmax(probs, axis=1)
        if probs[0, REJECT_LABEL] >= self.threshold_1 or probs[0, cmd] < self.threshold_2:
            cmd = np.array([REJECT_LABEL])
        '''
        '''
        if cmd_probs[0, cmd] < self.threshold_2:
            cmd = np.array([cmd_probs.shape[1]-1])
        '''
        '''
        else:
            cmd = np.argmax(probs, axis=1)
        '''

        return cmd, cmd_probs

    def parse_req(self, req):
        #signal = self.convert(req.data.data)
        signal = np.array(req.data.data, dtype=np.float32)
        cmd, probs = self.predict(signal)
        assert len(cmd) == 1
        cmd = int(cmd[0])
        probs = probs.tolist()[0]
        return ClassificationResponse(cmd, probs)

    def init_node(self):
        rospy.init_node('classifier')
        s = rospy.Service('classifier_service', Classification, self.parse_req)
        rospy.spin()



class MTLClassifier:
    def __init__(self, threshold_1, threshold_2, commands):
        self.commands = commands
        self.threshold_1 = threshold_1
        self.threshold_2 = threshold_2

        speakers = {"eng": 530, "ita": 502}
        labels_1 = [i for i in range(32)]
        labels_2 = [i for i in range(speakers[settings.input.language])]
        labels = list()
        task_n_labels = list((len(labels_1), len(labels_2)))

        assert settings.model.network in ["resnet8", "HS", "SS"]
        # base_path = "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/"
        base_path = Path(global_utils.get_curr_dir(__file__)).parent.joinpath(MODEL_PATH)
        if settings.model.network == "resnet8":
            ckpt_folder = os.path.join(base_path, "experiments", "MTL", settings.input.language, "SCR", "resnet8", "checkpoints")
            ckpt_name = os.listdir(path=ckpt_folder)[-1]
            ckpt_path = os.path.join(ckpt_folder, ckpt_name)
            self.cmd_model = ResNet8_PL(settings=settings, num_labels=len(labels_1), loss_weights=None).cuda()  # Load model
            ckpt = torch.load(ckpt_path, lambda s, l: s)
            state_dict = ckpt["state_dict"]
            del state_dict["loss_fn.weight"]
            self.cmd_model.load_state_dict(state_dict=state_dict)
            
            ckpt_folder = os.path.join(base_path, "experiments", "MTL", settings.input.language, "SI", "resnet8", "checkpoints")
            ckpt_name = os.listdir(path=ckpt_folder)[-1]
            ckpt_path = os.path.join(ckpt_folder, ckpt_name)
            self.spk_model = ResNet8_PL(settings=settings, num_labels=len(labels_2), loss_weights=None).cuda()  # Load model
            ckpt = torch.load(ckpt_path, lambda s, l: s)
            state_dict = ckpt["state_dict"]
            del state_dict["loss_fn.weight"]
            self.spk_model.load_state_dict(state_dict=state_dict)
        elif settings.model.network == "HS":
            ckpt_folder = os.path.join(base_path, "experiments", "MTL", settings.input.language, "SCR_SI", "HS_grad_norm", "checkpoints")
            ckpt_name = os.listdir(path=ckpt_folder)[-1]
            ckpt_path = os.path.join(ckpt_folder, ckpt_name)
            self.model = HardSharing_PL(settings=settings, task_n_labels=task_n_labels, task_loss_weights=np.array(object=(None, None))).cuda()
            ckpt = torch.load(ckpt_path, lambda s, l: s)
            state_dict = ckpt["state_dict"]
            self.model.load_state_dict(state_dict)
        elif settings.model.network == "SS":
            ckpt_folder = os.path.join(base_path, "experiments", "MTL", settings.input.language, "SCR_SI", "SS_grad_norm", "checkpoints")
            ckpt_name = os.listdir(path=ckpt_folder)[-1]
            ckpt_path = os.path.join(ckpt_folder, ckpt_name)
            self.model = SoftSharing_PL(settings=settings, task_n_labels=task_n_labels, task_loss_weights=np.array(object=(None, None))).cuda()
            ckpt = torch.load(ckpt_path, lambda s, l: s)
            state_dict = ckpt["state_dict"]
            self.model.load_state_dict(state_dict)
        
        # In order to avoid the waste of time related to the first inference
        melspectrogram = get_melspectrogram(waveform=np.zeros(shape=(4800)))
        self.model.predict(melspectrogram)
        #self.cmd_model.predict(melspectrogram)
        #self.spk_model.predict(melspectrogram)

        self.init_node()


    def _pcm2float(self, sound: np.ndarray):
        abs_max = np.abs(sound).max()
        sound = sound.astype('float32')
        if abs_max > 0:
            sound *= 1 / abs_max
        sound = sound.squeeze()  # depends on the use case
        return sound

    def _numpy2tensor(self, signal: np.ndarray):
        signal_size = signal.size
        signal_torch = torch.as_tensor(signal, dtype=torch.float32)
        signal_size_torch = torch.as_tensor(signal_size, dtype=torch.int64)
        return signal_torch, signal_size_torch

    def convert(self, signal):
        signal = np.array(signal)
        signal_nw = self._pcm2float(signal)
        return signal_nw

    def predict(self, signal: np.ndarray):
        melspectrogram = get_melspectrogram(waveform=signal)
        # prev_time = time.time()
        #cmd_probs = self.cmd_model.predict(melspectrogram)
        #spk_probs = self.spk_model.predict(melspectrogram)
        cmd_probs, spk_probs = self.model.predict(melspectrogram)
        # infer_time = time.time() - prev_time
        # print('\nINFER TIME: {}\n'.format(infer_time))
        cmd_probs = cmd_probs.cpu().detach().numpy()
        cmd = np.argmax(cmd_probs, axis=1)
        if len(cmd_probs[0]) < len(self.commands):
            cmd_probs = np.append(arr=cmd_probs, values=[1-cmd_probs[0][cmd]], axis=1)
        '''
        REJECT_LABEL = probs.shape[1] - 1
        # cmd = np.argmax(probs, axis=1)
        if probs[0, REJECT_LABEL] >= self.threshold_1 or probs[0, cmd] < self.threshold_2:
            cmd = np.array([REJECT_LABEL])
        '''
        '''
        if cmd_probs[0, cmd] < self.threshold_2:
            cmd = np.array([cmd_probs.shape[1]-1])
        '''
        '''
        else:
            cmd = np.argmax(probs, axis=1)
        '''

        return cmd, cmd_probs

    def parse_req(self, req):
        signal = self.convert(req.data.data)
        cmd, probs = self.predict(signal)
        assert len(cmd) == 1
        cmd = int(cmd[0])
        probs = probs.tolist()[0]
        return ClassificationResponse(cmd, probs)

    def init_node(self):
        rospy.init_node('classifier')
        s = rospy.Service('classifier_service', Classification, self.parse_req)
        rospy.spin()



if __name__ == "__main__":
    THRESHOLD_1 = 0.1     # 0.004
    THRESHOLD_2 = 0.7     # 0.999 - 0.9

    LANGUAGE = rospy.get_param("/language")
    DEMO = rospy.get_param("/demo")

    commands, ckpt_folder, ckpt_name = select_parameters(language=LANGUAGE, demo=DEMO)

    if LANGUAGE not in AVAILABLE_LANGS:
        raise Exception("Selected language not available.\nAvailable langs:", AVAILABLE_LANGS)
#    data_layer = AudioDataLayer(sample_rate=16000)
 #   data_loader = DataLoader(data_layer, batch_size=1, collate_fn=data_layer.collate_fn)
    #classifier = MTLClassifier(THRESHOLD_1, THRESHOLD_2, commands)
    #classifier = Classifier(LANGUAGE, THRESHOLD_1, THRESHOLD_2, commands, ckpt_folder, ckpt_name)
    classifier = NewClassifier(THRESHOLD_1, THRESHOLD_2, commands, ckpt_folder, ckpt_name)