'''
CREATE THE DATASET CSV FILES, one in english and one in italian language.

The dataset attributes are the followings.
    path:       path of the audio file
    type:       {real, sythetic, reject}
    subtype:    for real {smartphone, respeaker, ...}
                for synthetic {azure, google, naturalreaders, nemo, polly, vocalware}
                for reject {google_speech, mozilla_common_voice}
    speaker:    speaker ID
    label:    label of teh command pronunced inside the sample
'''

import os
import pandas as pd
from tqdm import tqdm
from commands import COMMANDS


HEADING = ["path", "type", "subtype", "speaker", "command"]
NOISE_HEADING = ["path", "type", "subtype"]
LANGS = ["eng", "ita"]
DATASET_NAME = "MIVIA_ISC_v2"
IN_PATH = os.path.join("datasets", DATASET_NAME)
OUT_DATASET = os.path.join("datasets", "MIVIA_ISC_v2")
OUT_PATH = os.path.join(OUT_DATASET, "annotations")

cmd_path = "commands"
syn_path = "synthetics"
rjt_path = "rejects"
noise_path = "noises"


def add_command_samples(data, lang_path, speaker, samples):
    type = "command"
    for sample in samples:
        sample_path = os.path.join(lang_path, sample)
        subtype = "telegram_bot" if len(sample.split('_'))==2 else "google"
        command = int(sample.split("_")[1].split(".")[0]) if sample.split("_")[1] not in LANGS else int(sample.split("_")[2].split(".")[0])
        data["path"].append(sample_path)
        data["type"].append(type)
        data["subtype"].append(subtype)
        data["speaker"].append(speaker)
        data["command"].append(command)
    return data

def add_synthetic_samples(data, lang_path, subtype, samples):
    type = "synthetic"

    for sample in samples:
        if ".wav" in sample:    # check if it is an audio file
            sample_path = os.path.join(lang_path, sample)
            
            temp = sample.split("_")
            speaker = temp[0]
            if "google" in subtype:
                speaker = "_".join(temp[0:3])
            elif "nemo" in subtype or "vocalware" in subtype:
                speaker = "_".join(temp[0:2])
            command = int(sample.split(".")[0].split("_")[-1].replace("cmd", ""))

            data["path"].append(sample_path)
            data["type"].append(type)
            data["subtype"].append(subtype)
            data["speaker"].append(speaker)
            data["command"].append(command)

    return data

def add_reject_samples(path, lang, data):
    rejects_df = pd.read_csv(path)
    data_iter = tqdm(rejects_df.index)
    for idx in data_iter:
        #subtype = "mozilla" if "common_voice" in sample else "google"
        #speaker = sample.split("_")[1] if subtype == "google" else sample.split("_")[3].split(".")[0]
        data["path"].append(rejects_df["path"][idx])
        data["type"].append(rejects_df["type"][idx])
        data["subtype"].append(rejects_df["subtype"][idx])
        data["speaker"].append(rejects_df["speaker"][idx])
        data["command"].append(len(COMMANDS)-1)
        data_iter.set_description("Working on REJECTS in {}".format(lang))
    return data


def get_noise_annotations_dict():
    noise_full_path = os.path.join(IN_PATH, noise_path)

    noises = {"path": [], "type": [], "subtype": []}
    noise_iter = tqdm(os.listdir(noise_full_path))
    for type in noise_iter:
        if ".wav" in type:
            path = os.path.join(noise_path, type)
            noises["path"].append(path)
            noises["type"].append("unknown")
            noises["subtype"].append("unknown")
        else:
            type_full_path = os.path.join(noise_full_path, type)
            for subtype in os.listdir(type_full_path):
                if ".wav" in subtype:
                    noises["path"].append(os.path.join(noise_path, type, subtype))
                    noises["type"].append(type)
                    noises["subtype"].append("unknown")
                else:
                    subtype_full_path = os.path.join(type_full_path,subtype)
                    for file in os.listdir(subtype_full_path):
                        noises["path"].append(os.path.join(noise_path, type, subtype, file))
                        noises["type"].append(type)
                        noises["subtype"].append(subtype)

        noise_iter.set_description("Working on NOISES") 
    
    return noises


data = dict()
for lang in LANGS:
    data[lang] = {"path":[], "type":[], "subtype":[], "speaker":[], "command":[]}

''' COMMAND SAMPLES '''
cmd_path = os.path.join(IN_PATH, cmd_path)
data_iter = tqdm(os.listdir(cmd_path))
for speaker in data_iter:
    speaker_path = os.path.join(cmd_path, speaker)
    for lang in os.listdir(speaker_path):
        lang_path = os.path.join(speaker_path, lang)
        try:
            samples = os.listdir(lang_path)
            data[lang] = add_command_samples(data[lang], lang_path.replace(IN_PATH, "."), speaker, samples)
        except FileNotFoundError:
            print("FileNotFound: {}".format(lang_path))
    data_iter.set_description("Working on COMMANDS")


''' SYNTHETIC SAMPLES '''
syn_path = os.path.join(IN_PATH, syn_path)
data_iter = tqdm(os.listdir(syn_path))
for service in data_iter:
    service_path = os.path.join(syn_path, service)
    for voice in os.listdir(service_path):
        if "." not in voice:    # check if it is a folder
            voice_path = os.path.join(service_path, voice)
            subtype = service + "_" + voice
            for lang in LANGS:
                lang_path = os.path.join(voice_path, lang)
                try:
                    samples = os.listdir(lang_path)
                    data[lang] = add_synthetic_samples(data[lang], lang_path.replace(IN_PATH, "."), subtype, samples)
                except FileNotFoundError:
                    print("FileNotFound: {}".format(lang_path))
    data_iter.set_description("Working on SYNTHETICS")        


''' REJECT SAMPLES '''
"""
for lang in LANGS:
    lang_path
    data[lang] = add_reject_samples(IN_PATH, lang, data[lang])
"""

''' WRITE CSV FILE '''
for lang in LANGS:
    out_path = os.path.join(OUT_PATH, lang)
    os.makedirs(name=out_path, exist_ok=True)
    out_file = os.path.join(out_path, "dataset.csv")
    df = pd.DataFrame(data=data[lang], columns=HEADING)
    df.to_csv(path_or_buf=out_file, index=False)

''' NOISE SAMPLES '''
"""
noise_data = get_noise_annotations_dict()
out_path = os.path.join(OUT_PATH,  "noise")
os.makedirs(name=out_path, exist_ok=True)
out_file = os.path.join(out_path, "dataset.csv")
df = pd.DataFrame(data=noise_data, columns=NOISE_HEADING)
df.to_csv(path_or_buf=out_file, index=False)
"""