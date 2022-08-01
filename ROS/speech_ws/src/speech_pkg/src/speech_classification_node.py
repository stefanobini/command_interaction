#!/usr/bin/python3

from model import Model
import numpy as np
from nemo.core.neural_types import NeuralType, AudioSignal, LengthsType
from nemo.core.classes import IterableDataset
from torch.utils.data import DataLoader
from speech_pkg.srv import Classification, ClassificationResponse
from settings import pepper, global_utils
import torch
import rospy
import sys
from pathlib import Path
import argparse
from lang_settings import AVAILABLE_LANGS
from colorama import Fore
from commands import DEMO3_CMD_ENG, DEMO3_CMD_ITA, DEMO7_CMD_ENG, DEMO7_CMD_ITA
import time


args = None

def select_parameters(args):
    models_path = Path(global_utils.get_curr_dir(__file__)).parent.joinpath("experiments")
    if args.demo == 3:
        if args.lang == 'eng':
            COMMANDS = DEMO3_CMD_ENG
            ckpt_folder = models_path.joinpath('eng', 'demo3_eng')
            ckpt_name = 'matchcboxnet--val_loss=2.2774-epoch=209.model' # demo3_eng
        elif args.lang == 'ita':
            COMMANDS = DEMO3_CMD_ITA
            ckpt_folder = models_path.joinpath('ita', 'demo3_ita')
            # ckpt_name = r"matchcboxnet--val_loss=1.4977-epoch=129.model"  # demo3_no_pretrain_ita
            # ckpt_name = r"matchcboxnet--val_loss=8.5081-epoch=184.model"  # demo3_pretrain_ita
            ckpt_name = 'matchcboxnet--val_loss=8.5081-epoch=184.model'   # demo3_ita

    elif args.demo == 7:
        if args.lang == 'eng':
            COMMANDS = DEMO7_CMD_ENG
            ckpt_folder = models_path.joinpath('eng', 'demo7_phase_I_eng')
            # ckpt_name = 'matchcboxnet--val_loss=1.9024-epoch=172.model' # demo7_eng
            
            # ckpt_name = 'matchcboxnet--val_loss=0.5461-epoch=172.model' # demo7_pretrain_eng

            ckpt_name = "matchcboxnet--val_loss=1.9424-epoch=50.model"  # demo7_phase_I_eng
        elif args.lang == 'ita':
            COMMANDS = DEMO7_CMD_ITA
            ckpt_folder = models_path.joinpath("ita", 'demo7_phase_I_ita')
            # ckpt_name = 'matchcboxnet--val_loss=2.7598-epoch=130.model'

            ckpt_name = "matchcboxnet--val_loss=3.2168-epoch=147.model" # demo7_phase_I_ita

            # ckpt = r"matchcboxnet--val_loss=2.7598-epoch=130.model"   # demo7_fix_ext
            # ckpt = r"matchcboxnet--val_loss=2.2616-epoch=103.model"   # demo7_no_pretrain_ext_ita
            # ckpt = r"matchcboxnet--val_loss=1.3722-epoch=202.model"   # demo7_no_pretrain_ita
            # ckpt = r"matchcboxnet--val_loss=0.7473-epoch=180.model"   # demo7_pretrain_ita
            # ckpt = r"matchcboxnet--val_loss=12.0919-epoch=80.model"   # demo7_pretrain_ext_ita
            # ckpt = r"matchcboxnet--val_loss=2.6342-epoch=380.model"   # demo7_fix_ext_google

            # ckpt = r"matchcboxnet--val_loss=0.2571-epoch=242.model"     # demo7_cmds
            # ckpt = r"matchcboxnet--val_loss=0.229-epoch=155.model"      # demo7_cmds_noise
            # ckpt = r"matchcboxnet--val_loss=3.0086-epoch=129.model"     # demo7_cmds_noise_reject
    
    return COMMANDS, ckpt_folder, ckpt_name


def infer_signal(model, signal):
    data_layer.set_signal(signal)
    batch = next(iter(data_loader))
    audio_signal, audio_signal_len = batch
    audio_signal, audio_signal_len = audio_signal.to(model.device), audio_signal_len.to(model.device)
    logits = model.forward(input_signal=audio_signal, input_signal_length=audio_signal_len)
    return logits

class AudioDataLayer(IterableDataset):
    @property
    def output_types(self):
        return {
            'audio_signal': NeuralType(('B', 'T'), AudioSignal(freq=self._sample_rate)),
            'a_sig_length': NeuralType(tuple('B'), LengthsType()),
        }

    def __init__(self, sample_rate):
        super().__init__()
        self._sample_rate = sample_rate
        self.output = True

    def __iter__(self):
        return self

    def __next__(self):
        if not self.output:
            raise StopIteration
        self.output = False
        return torch.as_tensor(self.signal, dtype=torch.float32), \
               torch.as_tensor(self.signal_shape, dtype=torch.int64)

    def set_signal(self, signal):
        self.signal = signal.astype(np.float32)
        self.signal_shape = self.signal.size
        self.output = True

    def __len__(self):
        return 1

class Classifier:
    def __init__(self, lang, threshold_1, threshold_2, commands, ckpt_folder, ckpt_name):
        self.commands = commands
        self.threshold_1 = threshold_1
        self.threshold_2 = threshold_2

        self.model = Model.load_backup(exp_dir=ckpt_folder, ckpt_name=ckpt_name)
        print("# Loaded model lang:", lang)
        print("# Model loaded:", ckpt_folder)
        # self.model = self.load_model(lang)

        self.model = self.model.eval()
        if torch.cuda.is_available():
            self.model = self.model.cuda()
        else:
            self.model = self.model.cpu()
        
        # In order to avoid the waste of time related to the first inference
        logits = infer_signal(self.model, np.zeros(shape=(20000,)))
        self.model.predict(logits)

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

    def predict_cmd(self, signal: np.ndarray):
        logits = infer_signal(self.model, signal)
        # prev_time = time.time()
        probs = self.model.predict(logits)
        # infer_time = time.time() - prev_time
        # print('\nINFER TIME: {}\n'.format(infer_time))
        probs = probs.cpu().detach().numpy()
        cmd = np.argmax(probs, axis=1)
        if len(probs[0]) < len(self.commands):
            probs = np.append(arr=probs, values=[1-probs[0][cmd]], axis=1)
        '''
        REJECT_LABEL = probs.shape[1] - 1
        # cmd = np.argmax(probs, axis=1)
        if probs[0, REJECT_LABEL] >= self.threshold_1 or probs[0, cmd] < self.threshold_2:
            cmd = np.array([REJECT_LABEL])
        '''
        if probs[0, cmd] < self.threshold_2:
            cmd = np.array([probs.shape[1]-1])
        '''
        else:
            cmd = np.argmax(probs, axis=1)
        '''

        return cmd, probs

    def parse_req(self, req):
        signal = self.convert(req.data.data)
        cmd, probs = self.predict_cmd(signal)
        assert len(cmd) == 1
        cmd = int(cmd[0])
        probs = probs.tolist()[0]
        return ClassificationResponse(cmd, probs)

    def init_node(self):
        rospy.init_node('classifier')
        s = rospy.Service('classifier_service', Classification, self.parse_req)
        rospy.spin()

    def load_model(self, lang, ckpt_folder, ckpt_name):
        model = Model.load_backup(exp_dir=ckpt_folder, ckpt_name=ckpt_name)
        print("# Loaded model lang:", lang)
        print("# Model loaded:", ckpt_folder)
        
        return model

if __name__ == "__main__":
    THRESHOLD_1 = 0.001     # 0.004
    THRESHOLD_2 = 0.999     # 0.999

    parser = argparse.ArgumentParser()
    parser.add_argument("--lang", required=True, dest="lang", type=str)
    parser.add_argument("--demo", required=True, dest="demo", type=int)
    args, unknown = parser.parse_known_args(args=rospy.myargv(argv=sys.argv)[1:])

    commands, ckpt_folder, ckpt_name = select_parameters(args=args)

    if args.lang not in AVAILABLE_LANGS:
        raise Exception("Selected lang not available.\nAvailable langs:", AVAILABLE_LANGS)
    data_layer = AudioDataLayer(sample_rate=16000)
    data_loader = DataLoader(data_layer, batch_size=1, collate_fn=data_layer.collate_fn)
    classifier = Classifier(args.lang, THRESHOLD_1, THRESHOLD_2, commands, ckpt_folder, ckpt_name)