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

SRC_DATASET = "MSIexp1"
SPEECH_HEADING = ["path", "type", "subtype", "speaker", "command", "noise_path", "noise_type", "noise_subtype", "snr"]
NOISE_HEADING = ["path", "type", "subtype", "speaker", "command", "noise_path", "noise_type", "noise_subtype", "snr"]
FULL_HEADING = ["path", "type", "subtype", "speaker", "command", "noise_path", "noise_type", "noise_subtype", "snr"]
SRC_DATASET_PATH = ""
OUT_PATH = ""
LANGs = list()
SPEECH_HEADING, NOISE_HEADING, FULL_HEADING = list(), list(), list()
SNRs = range(settings.noise.min_snr, settings.noise.max_snr+settings.noise.snr_step, settings.noise.snr_step)

if "MIVIA_ISC_v1" in SRC_DATASET:
    SRC_DATASET_PATH = os.path.join("datasets", "MIVIA_ISC_v1")
    OUT_PATH = os.path.join("datasets", "SCR_experimentation")
    LANGs = ["eng", "ita"]
elif "MSIexp0" in SRC_DATASET:
    SRC_DATASET_PATH = os.path.join("datasets", SRC_DATASET)
    OUT_PATH = os.path.join("datasets", SRC_DATASET)
    LANGs = ["eng", "esp", "ita"]
elif "MSIexp1" in SRC_DATASET:
    SRC_DATASET_PATH = os.path.join("datasets", SRC_DATASET)
    OUT_PATH = os.path.join("datasets", SRC_DATASET)
    LANGs = ["eng", "esp", "ita"]
    SPEECH_HEADING = ["path", "type", "subtype", "speaker", "intent", "explicit", "implicit", "noise_path", "noise_type", "noise_subtype", "snr"]
    NOISE_HEADING = ["path", "type", "subtype", "speaker", "intent", "explicit", "implicit", "noise_path", "noise_type", "noise_subtype", "snr"]
    FULL_HEADING = ["path", "type", "subtype", "speaker", "intent", "explicit", "implicit", "noise_path", "noise_type", "noise_subtype", "snr"]


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
            
            speech_data["path"].append(speech_path)
            speech_data["type"].append(speech_type)
            speech_data["subtype"].append(speech_subtype)
            speech_data["speaker"].append(speaker)
            speech_data["command"].append(command)
            full_speech_path = os.path.join(out_path, os.path.split(speech_path)[0])
            os.makedirs(full_speech_path, exist_ok=True)
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
                noisy_speech_path = speech_path.split('.')[idx][idx:] + "_snr" + str(snr) + '.' + speech_path.split('.')[idx+1]
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
#build_set(subset="testing")