"""
Build dataset reading the annotation files and creating the audio files.
Training file are just copied, while on validation and test one the noise was applied.
"""

import os
import shutil
import pandas as pd
from tqdm import tqdm
import torch
import torchaudio

from utils.preprocessing import Preprocessing
from utils.settings.SCR_conf import settings
from intents import INTENTS_DICT_MSIEXP0, INTENTS_DICT_MSIEXP1


DATASET_NAME = "MSI_exp0"
SRC_DATASET_PATH = os.path.join("datasets", DATASET_NAME)
OUT_PATH = os.path.join("datasets", DATASET_NAME)
LANGs = ["eng", "esp", "ita"]
SPEECH_HEADING = ["path", "type", "subtype", "speaker", "command", "noise_path", "noise_type", "noise_subtype", "snr"]
NOISE_HEADING = ["path", "type", "subtype", "speaker", "command", "noise_path", "noise_type", "noise_subtype", "snr"]
FULL_HEADING = ["path", "type", "subtype", "speaker", "command", "noise_path", "noise_type", "noise_subtype", "snr"]
SNRs = range(settings.noise.min_snr, settings.noise.max_snr+settings.noise.snr_step, settings.noise.snr_step)
intents = INTENTS_DICT_MSIEXP0 if DATASET_NAME=="MSI_exp0" else INTENTS_DICT_MSIEXP1
INTENTs:dict = {"left":{"eng":0, "esp":len(intents)-1, "ita":len(intents)-1,}, "right":{"eng":2, "esp":len(intents)-1, "ita":len(intents)-1,}, "stop":{"eng":3, "esp":3, "ita":3}, "chatter":len(intents)-1,}


def get_item(speech_annotations, noise_annotations, index, preprocess):
    speech_item = speech_annotations.iloc[index]
    rel_speech_path = speech_item.path
    speech_path = os.path.join(SRC_DATASET_PATH, rel_speech_path)
    speech, speech_sample_rate = torchaudio.load(speech_path)
    speech = preprocess.resample_audio(waveform=speech, sample_rate=speech_sample_rate)  # uniform sample rate
    speech = torch.mean(input=speech, dim=0, keepdim=True)  # reduce to one channel

    noise_index = index % len(noise_annotations)
    noise_item = noise_annotations.iloc[noise_index]
    rel_noise_path = noise_item.path
    noise_path = os.path.join(SRC_DATASET_PATH, rel_noise_path)
    noise, noise_sample_rate = torchaudio.load(noise_path)
    noise = preprocess.resample_audio(waveform=noise, sample_rate=noise_sample_rate)     # uniform sample rate
    noise = torch.mean(input=noise, dim=0, keepdim=True)    # reduce to one channel

    return speech, settings.input.sample_rate, speech_item.path, speech_item.type, speech_item.subtype, speech_item.speaker, int(speech_item.command), noise, noise_item.path, noise_item.type, noise_item.subtype


def build_train_set():
    preprocess = Preprocessing(settings=settings)
    out_path = os.path.join(OUT_PATH, "training")
    for lang in LANGs:
        speech_annotations = pd.read_csv(os.path.join(OUT_PATH, "annotations", lang, "training.csv"), sep=',')
        noise_annotations = pd.read_csv(os.path.join(OUT_PATH, "annotations", "noise", "training.csv"), sep=',')

        speech_data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}
        noise_data = {"path": list(), "type": list(), "subtype": list()}
        ann_iter = tqdm(range(len(speech_annotations)))
        for idx in ann_iter:
            speech, sample_rate, speech_path, speech_type, speech_subtype, speaker, command, noise, noise_path, noise_type, noise_subtype = get_item(speech_annotations=speech_annotations, noise_annotations=noise_annotations, index=idx, preprocess=preprocess)
            # for manage the same speaker that pronunced different commands in Google Speech Commands dataset
            google_cmd = speech_path.split("/")[2]
            speech_path = speech_path.replace("{}/".format(google_cmd), "{}_".format(google_cmd)) if speech_subtype == "google_speech" else speech_path

            speech_path = os.path.join("rejects", lang, os.path.split(speech_path)[1]) if speech_type == "reject" else speech_path.replace("./", "")
            
            # for google speech commands with commands included in our dictionary
            if speech_subtype == "google_speech" and google_cmd in INTENTs:
                speech_path = speech_path.replace(".google_speech_commands_v1", os.path.join("rejects", lang))
                command = INTENTs[google_cmd][lang]
                speech_type = "reject" if command == 3 else "command"

            speech_data["path"].append(speech_path)
            speech_data["type"].append(speech_type)
            speech_data["subtype"].append(speech_subtype)
            speech_data["speaker"].append(speaker)
            speech_data["command"].append(command)
            folder_path = os.path.join(out_path, os.path.split(speech_path)[0])
            os.makedirs(folder_path, exist_ok=True)
            torchaudio.save(os.path.join(out_path, speech_path), speech, sample_rate)

            if noise_path not in noise_data["path"]:
                noise_data["path"].append(noise_path)
                noise_data["type"].append(noise_type)
                noise_data["subtype"].append(noise_subtype)
                full_noise_path = os.path.join(out_path, os.path.split(noise_path)[0])
                os.makedirs(full_noise_path, exist_ok=True)
                torchaudio.save(os.path.join(out_path, noise_path), noise, sample_rate)

            ann_iter.set_description("Building '{}-training' set".format(lang))

        out_folder = os.path.join(out_path, "annotations", lang)
        if not os.path.isdir(out_folder):
            os.makedirs(out_folder)
        out_file = os.path.join(out_folder, "training.csv")
        df = pd.DataFrame(data=speech_data, columns=SPEECH_HEADING)
        df.to_csv(path_or_buf=out_file, index=False)

        out_folder = os.path.join(out_path, "annotations", "noise")
        if not os.path.isdir(out_folder):
            os.makedirs(out_folder)
        out_file = os.path.join(out_folder, "training.csv")
        df = pd.DataFrame(data=noise_data, columns=NOISE_HEADING)
        df.to_csv(path_or_buf=out_file, index=False)


def build_set(subset:str):
    preprocess = Preprocessing(settings=settings)
    out_path = os.path.join(OUT_PATH, subset)
    for lang in LANGs:
        speech_annotations = pd.read_csv(os.path.join(OUT_PATH, "annotations", lang, "{}.csv".format(subset)), sep=',')
        noise_annotations = pd.read_csv(os.path.join(OUT_PATH, "annotations", "noise", "{}.csv".format(subset)), sep=',')

        data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list(), "noise_path": list(), "noise_type": list(), "noise_subtype": list(), "snr": list()}
        ann_iter = tqdm(range(len(speech_annotations)))
        for idx in ann_iter:
            speech, sample_rate, speech_path, speech_type, speech_subtype, speaker, command, noise, noise_path, noise_type, noise_subtype = get_item(speech_annotations=speech_annotations, noise_annotations=noise_annotations, index=idx, preprocess=preprocess)
            for snr in SNRs:
                noisy_speech = preprocess.get_noisy_speech(speech=speech, noise=noise, snr_db=snr) if snr is not None else speech
                idx = 1 if speech_path[0] == '.' else 0 # if the path begin with './'
                noisy_speech_path = speech_path.replace(speech_path.split('.')[-1], "_snr"+str(snr)+"."+speech_path.split('.')[-1])
                
                # for manage the same speaker that pronunced different commands in Google Speech Commands dataset
                google_cmd = noisy_speech_path.split("/")[2]
                noisy_speech_path = noisy_speech_path.replace("{}/".format(google_cmd), "{}_".format(google_cmd)) if speech_subtype == "google_speech" else noisy_speech_path
                
                noisy_speech_path = os.path.join("rejects", lang, os.path.split(noisy_speech_path)[1]) if speech_type == "reject" else noisy_speech_path.replace("./", "")
                
                # for google speech commands with commands included in our dictionary
                if speech_subtype == "google_speech" and google_cmd in INTENTs:
                    noisy_speech_path = noisy_speech_path.replace(".google_speech_commands_v1", os.path.join("rejects", lang))
                    command = INTENTs[google_cmd][lang]
                    speech_type = "reject" if command == 3 else "command"

                data["path"].append(noisy_speech_path)
                data["type"].append(speech_type)
                data["subtype"].append(speech_subtype)
                data["speaker"].append(speaker)
                data["command"].append(command)
                data["noise_path"].append(noise_path)
                data["noise_type"].append(noise_type)
                data["noise_subtype"].append(noise_subtype)
                data["snr"].append(snr)
                full_path = os.path.join(out_path, os.path.split(noisy_speech_path)[0])
                os.makedirs(full_path, exist_ok=True)
                torchaudio.save(os.path.join(out_path, noisy_speech_path), noisy_speech, sample_rate)

            ann_iter.set_description("Building '{}-{}' set".format(lang, subset))

        out_folder = os.path.join(out_path, "annotations", lang)
        os.makedirs(out_folder, exist_ok=True)
        out_file = os.path.join(out_folder, "{}.csv".format(subset))
        df = pd.DataFrame(data=data, columns=FULL_HEADING)
        df.to_csv(path_or_buf=out_file, index=False)

        out_folder = os.path.join(out_path, "annotations", "noise")
        os.makedirs(out_folder, exist_ok=True)
        out_file = os.path.join(out_folder, "{}.csv".format(subset))
        shutil.copyfile(src=os.path.join(OUT_PATH, "annotations", "noise", "{}.csv".format(subset)), dst=out_file)


build_train_set()
build_set(subset="validation")
build_set(subset="testing")