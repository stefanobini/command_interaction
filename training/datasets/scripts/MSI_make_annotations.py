'''
CREATE THE DATASET CSV FILES, one in english and one in italian language.

The dataset attributes are the followings.
    path:       path of the audio file
    type:       {real, sythetic, reject}
    subtype:    for real {smartphone, respeaker, ...}
                for synthetic {azure, google, naturalreaders, nemo, polly, vocalware}
                for reject {google_speech, mozilla_common_voice}
    speaker:    speaker ID
    int:        label for inten, e.g., "go left (urgent)" and "left (urgent)" have the same label, but different from "go left (not urgent)". Valued in range [0-22]
    exp_int:    label for explicit intent, e.g., "go left (urgent)" and "go left (not urgent)" have the same label, but different from "left (urgent)". valued in range [0-N], where N depends from the language
    imp_int:    label for implicit intent. Valued in {0:not urgent, 1:urgent}
'''

import os
import pandas as pd
from tqdm import tqdm
from intents import INTENTS, EXPLICIT_INTENTS, IMPLICIT_INTENTS


HEADING = ["path", "type", "subtype", "speaker", "intent", "explicit", "implicit"]
NOISE_HEADING = ["path", "type", "subtype"]
LANGS = ["eng", "esp", "ita"]
IN_DATASET_NAME = "MSI"
IN_DATASET_PATH = os.path.join("datasets", IN_DATASET_NAME)
REJECT_ANNOTATION_PATH = os.path.join("datasets", "google_speech_commands_v1", "google_annotations.csv")
OUT_PATH = os.path.join(IN_DATASET_PATH, "annotations")

cmd_path = "commands"
# syn_path = "synthetics"
rjt_path = "rejects"
noise_path = "noises"


def add_command_samples(data, lang_path, lang, speaker, samples):
    type = "command"
    for sample in samples:
        sample_path = os.path.join(lang_path, sample)
        subtype = "telegram_bot"
        labels = sample.split("_")
        intent = int(labels[0])
        explicit = INTENTS[intent]["explicit"][lang][int(labels[1])]["id"]
        implicit = INTENTS[intent]["implicit"][lang]["id"]
        data["path"].append(sample_path)
        data["type"].append(type)
        data["subtype"].append(subtype)
        data["speaker"].append(speaker)
        data["intent"].append(intent)
        data["explicit"].append(explicit)
        data["implicit"].append(implicit)
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
            elif "nemo" in subtype:
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
        data["path"].append(os.path.join("google_speech_commands_v1", rejects_df["path"][idx]))
        data["type"].append(rejects_df["type"][idx])
        data["subtype"].append(rejects_df["subtype"][idx])
        data["speaker"].append(rejects_df["speaker"][idx])
        data["intent"].append(None)
        data["explicit"].append(None)
        data["implicit"].append(None)
        data_iter.set_description("Working on REJECTS in {}".format(lang))
    return data


def get_noise_annotations_dict():
    noise_full_path = os.path.join(IN_DATASET_PATH, noise_path)
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
    data[lang] = {"path":[], "type":[], "subtype":[], "speaker":[], "intent":[], "explicit":[], "implicit":[]}

''' COMMAND SAMPLES '''
cmd_path = os.path.join(IN_DATASET_PATH, cmd_path)
data_iter = tqdm(os.listdir(cmd_path))
for speaker in data_iter:
    speaker_path = os.path.join(cmd_path, speaker)
    for lang in os.listdir(speaker_path):
        lang_path = os.path.join(speaker_path, lang)
        try:
            samples = os.listdir(lang_path)
            data[lang] = add_command_samples(data[lang], lang_path.replace(IN_DATASET_PATH, "."), lang, speaker, samples)
        except FileNotFoundError:
            print("FileNotFound: {}".format(lang_path)) 
    data_iter.set_description("Working on COMMANDS")

''' SYNTHETIC SAMPLES '''
"""
syn_full_path = os.path.join(DATASET_PATH, syn_path)
data_iter = tqdm(os.listdir(syn_full_path))
for service in data_iter:
    print(service)
    service_path = os.path.join(syn_path, service)
    syn_full_service_path = os.path.join(syn_full_path, service)
    for voice in os.listdir(syn_full_service_path):
        if "." not in voice:    # check if it is a folder
            voice_path = os.path.join(service_path, voice)
            syn_full_voice_path = os.path.join(syn_full_service_path, voice)
            subtype = service + "_" + voice

            # English language
            lang = LANGS[0]
            lang_path = os.path.join(voice_path, lang)
            syn_full_lang_path = os.path.join(syn_full_voice_path, lang)
            try:
                samples = os.listdir(syn_full_lang_path)
                eng_data = add_synthetic_samples(eng_data, lang_path, subtype, samples)
            except FileNotFoundError:
                print("FileNotFound: {}".format(syn_full_lang_path))
            
            # Italian language
            lang = LANGS[1]
            lang_path = os.path.join(voice_path, lang)
            syn_full_lang_path = os.path.join(syn_full_voice_path, lang)
            try:
                samples = os.listdir(syn_full_lang_path)
                ita_data = add_synthetic_samples(ita_data, lang_path, subtype, samples)
            except FileNotFoundError:
                print("FileNotFound: {}".format(syn_full_lang_path))
            
    data_iter.set_description("Working on SYNTHETICS")        
"""

''' REJECT SAMPLES '''
"""
for lang in LANGS:
    data[lang] = add_reject_samples(REJECT_ANNOTATION_PATH, lang, data[lang])
"""

''' NOISE SAMPLES '''
# noise_data = get_noise_annotations_dict()

''' WRITE CSV FILE '''
for lang in LANGS:  
    out_path =  os.path.join(OUT_PATH, lang)
    os.makedirs(out_path, exist_ok=True)
    out_file = os.path.join(out_path, "dataset.csv")
    df = pd.DataFrame(data=data[lang], columns=HEADING)
    df.to_csv(path_or_buf=out_file, index=False)
"""
out_file = os.path.join(OUT_PATH, "noise", "dataset.csv")
df = pd.DataFrame(data=noise_data, columns=NOISE_HEADING)
df.to_csv(path_or_buf=out_file, index=False)
"""