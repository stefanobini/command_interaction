import os
import pandas as pd
from tqdm import tqdm
import torch
import torchaudio

import preprocessing


def get_item(index):
    speech_item = speech_annotations.iloc[index]
    rel_speech_path = speech_item.path
    speech_path = os.path.join(SRC_DATASET_PATH, rel_speech_path)
    speech, speech_sample_rate = torchaudio.load(speech_path)
    # print(speech.shape)
    # speech = preprocessing.resample_audio(waveform=speech, sample_rate=speech_sample_rate)  # uniform sample rate
    # print(speech.shape)
    # print("\n")
    speech = torch.mean(input=speech, dim=0, keepdim=True)  # reduce to one channel

    noise_index = index % len(noise_annotations)
    noise_item = noise_annotations.iloc[noise_index]
    rel_noise_path = noise_item.path
    noise_path = os.path.join(SRC_DATASET_PATH, rel_noise_path)
    noise, noise_sample_rate = torchaudio.load(noise_path)
    # noise = preprocessing.resample_audio(waveform=noise, sample_rate=noise_sample_rate)     # uniform sample rate
    noise = torch.mean(input=noise, dim=0, keepdim=True)    # reduce to one channel

    return speech, speech_sample_rate, speech_item.path, speech_item.type, speech_item.subtype, speech_item.speaker, int(speech_item.label), noise, noise_item.path, noise_item.type, noise_item.subtype


SRC_DATASET_PATH = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1"
OUT_PATH = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/final_dataset/validation"
LANGs = ["ita", "eng"]
HEADING = ["path", "type", "subtype", "speaker", "label", "noise_path", "noise_type", "noise_subtype"]
MIN_SNR = -10
MAX_SNR = 40
SNR_STEP = 5
SNRs = range(MIN_SNR, MAX_SNR+SNR_STEP, SNR_STEP)

'''
preprocess = dict()
for snr in SNRs:
    preprocess[snr] = preprocessing.Preprocessing(snr=snr)
'''
preprocess = preprocessing.Preprocessing()

for lang in LANGs:
    speech_annotations = pd.read_csv(os.path.join(SRC_DATASET_PATH, "annotations", lang, "validation.csv"), sep=',')
    noise_annotations = pd.read_csv(os.path.join(SRC_DATASET_PATH, "annotations", "noise", "validation.csv"), sep=',')

    data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "label": list(), "noise_path": list(), "noise_type": list(), "noise_subtype": list(), "snr": list()}
    ann_iter = tqdm(range(len(speech_annotations)))
    for idx in ann_iter:
        speech, sample_rate, speech_path, speech_type, speech_subtype, speaker, label, noise, noise_path, noise_type, noise_subtype = get_item(index=idx)
        for snr in SNRs:
            noisy_speech = preprocess.get_noisy_speech(speech=speech, noise=noise, snr_db=snr) if snr is not None else speech
            noisy_speech_path = speech_path.split('.')[0] + "_snr" + str(snr) + '.' + speech_path.split('.')[1]
            data["path"].append(noisy_speech_path)
            data["type"].append(speech_type)
            data["subtype"].append(speech_subtype)
            data["speaker"].append(speaker)
            data["label"].append(label)
            data["noise_path"].append(noise_path)
            data["noise_type"].append(noise_type)
            data["noise_subtype"].append(noise_subtype)
            data["snr"].append(snr)
            full_path = os.path.join(OUT_PATH, os.path.split(noisy_speech_path)[0])
            
            if not os.path.exists(full_path):
                os.makedirs(full_path)
            torchaudio.save(os.path.join(OUT_PATH, noisy_speech_path), noisy_speech, sample_rate)

        ann_iter.set_description("Building {} validation set".format(lang))

    out_file = os.path.join(OUT_PATH, "annotations", lang, "noisy_validation.csv")
    df = pd.DataFrame(data=data, columns=HEADING)
    df.to_csv(path_or_buf=out_file, index=False)