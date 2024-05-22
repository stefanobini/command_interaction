#!/usr/bin/python3
import os
import numpy as np
import torch
import rospy
from pathlib import Path
import colorama
colorama.init(autoreset=True)
from colorama import Fore, Back
import librosa
from PIL import Image
import time
import torchaudio
from dotmap import DotMap

from commands import MAPPING, AVAILABLE_LANGS
from speech_pkg.srv import SCR, SCRResponse
from settings import pepper, global_utils

#from mtl_exp.MTL_conf import settings
from settings.model_conf import settings
from models.resnet8 import ResNet8_PL
from models.mobilenetv2 import MobileNetV2_PL
from models.conformer import Conformer_PL


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
        return spectrogram_db

def preprocess(waveform: np.ndarray):
    waveform = np.expand_dims(waveform, axis=0) # add channel dimension
    waveform = torch.tensor(waveform, dtype=torch.float32).cuda()
    melspectrogram = get_melspectrogram(waveform=waveform)
    db_melspectrogram = amplitude_to_db_spectrogram(spectrogram=melspectrogram)
    return db_melspectrogram

def infer_signal(model, signal):
    data_layer.set_signal(signal)
    batch = next(iter(data_loader))
    audio_signal, audio_signal_len = batch
    audio_signal, audio_signal_len = audio_signal.to(model.device), audio_signal_len.to(model.device)
    logits = model.forward(input_signal=audio_signal, input_signal_length=audio_signal_len)
    return logits


class Classifier:
    def __init__(self, n_commands, ckpt_folder, ckpt_name):
        assert settings.model.network in ["resnet8", "conformer", "mobilenetv2"]
        ckpt_path = os.path.join(ckpt_folder, ckpt_name)
        print(Back.YELLOW + ckpt_path)
	# Model Selection
        if settings.model.network == "resnet8":
                self.model = ResNet8_PL(settings=settings, num_labels=n_commands, loss_weights=None).cuda()
        elif settings.model.network == "mobilenetv2":
                self.model = MobileNetV2_PL(settings=settings, num_labels=n_commands, loss_weights=None).cuda()
        elif settings.model.network == "conformer":
                self.model = Conformer_PL(settings=settings, num_labels=n_commands, loss_weights=None).cuda()
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
        prev_time = time.time()
        #cmd_probs = self.cmd_model.predict(melspectrogram)
        #spk_probs = self.spk_model.predict(melspectrogram)
        cmd_probs = self.model.predict(x)
        infer_time = time.time() - prev_time
        print('INFER TIME: {:.4f}'.format(infer_time))
        cmd_probs = cmd_probs.cpu().detach().numpy()
        cmd = np.argmax(cmd_probs, axis=1)
        return cmd, cmd_probs

    def parse_req(self, req):
        signal = np.array(req.data.data, dtype=np.float32)
        cmd, probs = self.predict(signal)
        assert len(cmd) == 1
        cmd = int(cmd[0])
        probs = probs.tolist()[0]
        return SCRResponse(cmd, probs)

    def init_node(self):
        rospy.init_node('scr_node')
        s = rospy.Service('scr_service', SCR, self.parse_req)
        rospy.spin()


class MTLClassifier:
    def __init__(self, n_commands):
        self.n_commands = n_commands

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
        if len(cmd_probs[0]) < self.n_commands:
            cmd_probs = np.append(arr=cmd_probs, values=[1-cmd_probs[0][cmd]], axis=1)
        return cmd, cmd_probs

    def parse_req(self, req):
        signal = self.convert(req.data.data)
        cmd, probs = self.predict(signal)
        assert len(cmd) == 1
        cmd = int(cmd[0])
        probs = probs.tolist()[0]
        return SCRResponse(cmd, probs)

    def init_node(self):
        rospy.init_node('scr_node')
        s = rospy.Service('scr_service', SCR, self.parse_req)
        rospy.spin()


if __name__ == "__main__":
    LANGUAGE = rospy.get_param("/language")
    DEMO = rospy.get_param("/demo")

    n_commands = len(MAPPING[DEMO])
    models_path = Path(global_utils.get_curr_dir(__file__)).parent.joinpath(settings.logger.folder)
    ckpt_folder = models_path.joinpath('demo'+DEMO, LANGUAGE, settings.model.network, settings.noise.curriculum_learning.distribution)
    ckpt_folder = models_path.joinpath(ckpt_folder, "checkpoints")
    ckpt_name = os.listdir(path=ckpt_folder)[-1]

    if LANGUAGE not in AVAILABLE_LANGS:
        raise Exception("Selected language not available.\nAvailable langs:", AVAILABLE_LANGS)
    #classifier = MTLClassifier(n_commands)
    classifier = Classifier(n_commands, ckpt_folder, ckpt_name)
