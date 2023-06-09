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


HEADING = ["path", "type", "subtype", "speaker", "label"]
NOISE_HEADING = ["path", "type", "subtype"]
LANGS = ["eng", "ita"]
DATASET_NAME = "MIVIA_ISC_v1"
DATASET_PATH = os.path.join("datasets", DATASET_NAME)
OUT_PATH = os.path.join(DATASET_PATH, "annotations")

cmd_path = "commands"
syn_path = "synthetics"
rjt_path = "rejects"
noise_path = "noises"


def add_command_samples(data, lang_path, speaker, samples):
    type = "command"

    for sample in samples:
        sample_path = os.path.join(lang_path, sample)
        subtype = "telegram_bot"
        command = int(sample.split("_")[1].split(".")[0]) if sample.split("_")[1] not in LANGS else int(sample.split("_")[2].split(".")[0])

        data["path"].append(sample_path)
        data["type"].append(type)
        data["subtype"].append(subtype)
        data["speaker"].append(speaker)
        data["label"].append(command)

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
            data["label"].append(command)

    return data

def add_reject_samples(dataset_path, rjt_path, lang, data):
    type = "reject"

    lang_path = os.path.join(rjt_path, lang)
    rjt_full_path = os.path.join(dataset_path, rjt_path, lang)

    data_iter = tqdm(os.listdir(rjt_full_path))
    for sample in data_iter:
        sample_path = os.path.join(lang_path, sample)
        subtype = "mozilla_common_voice" if "common_voice" in sample else "google_speech"
        speaker = sample.split("_")[1] if subtype == "google_speech" else sample.split("_")[3].split(".")[0]
        command = 24 if ("google_speech" in subtype and "stop" in sample) else 31
        type = "command" if (subtype == "google_speech" and "stop" in sample) else "reject"

        data["path"].append(sample_path)
        data["type"].append(type)
        data["subtype"].append(subtype)
        data["speaker"].append(speaker)
        data["label"].append(command)

        data_iter.set_description("Working on REJECTS in {}".format(lang))
    
    return data


def get_noise_annotations_dict():
    noise_full_path = os.path.join(DATASET_PATH, noise_path)

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

#"""
eng_data = {"path": [], "type": [], "subtype": [], "speaker": [], "label": []}
ita_data = {"path": [], "type": [], "subtype": [], "speaker": [], "label": []}

''' COMMAND SAMPLES '''
cmd_full_path = os.path.join(DATASET_PATH, cmd_path)
data_iter = tqdm(os.listdir(cmd_full_path))
for speaker in data_iter:
    speaker_path = os.path.join(cmd_path, speaker)
    cmd_full_speaker_path = os.path.join(cmd_full_path, speaker)

    # English language
    lang = LANGS[0]
    lang_path = os.path.join(speaker_path, lang)
    cmd_full_lang_path = os.path.join(cmd_full_speaker_path, lang)
    try:
        samples = os.listdir(cmd_full_lang_path)
        eng_data = add_command_samples(eng_data, lang_path, speaker, samples)
    except FileNotFoundError:
        print("FileNotFound: {}".format(cmd_full_lang_path))
    
    # Italian language
    lang = LANGS[1]
    lang_path = os.path.join(speaker_path, lang)
    cmd_full_lang_path = os.path.join(cmd_full_speaker_path, lang)
    try:
        samples = os.listdir(cmd_full_lang_path)
        ita_data = add_command_samples(ita_data, lang_path, speaker, samples)
    except FileNotFoundError:
        print("FileNotFound: {}".format(cmd_full_lang_path))
    
    data_iter.set_description("Working on COMMANDS")


''' SYNTHETIC SAMPLES '''
syn_full_path = os.path.join(DATASET_PATH, syn_path)
data_iter = tqdm(os.listdir(syn_full_path))
for service in data_iter:
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


''' REJECT SAMPLES '''
# English language
lang = LANGS[0]
eng_data = add_reject_samples(DATASET_PATH, rjt_path, lang, eng_data)

# Italian language
lang = LANGS[1]
ita_data = add_reject_samples(DATASET_PATH, rjt_path, lang, ita_data)
   

''' WRITE CSV FILE '''
lang = LANGS[0]
out_path = os.path.join(OUT_PATH, lang)
os.makedirs(name=out_path, exist_ok=True)
out_file = os.path.join(out_path, "dataset.csv")
df = pd.DataFrame(data=eng_data, columns=HEADING)
df.to_csv(path_or_buf=out_file, index=False)

lang = LANGS[1]
out_path = os.path.join(OUT_PATH, lang)
os.makedirs(name=out_path, exist_ok=True)
out_file = os.path.join(out_path, "dataset.csv")
df = pd.DataFrame(data=ita_data, columns=HEADING)
df.to_csv(path_or_buf=out_file, index=False)
#"""

''' NOISE SAMPLES '''
noise_data = get_noise_annotations_dict()
out_path = os.path.join(OUT_PATH,  "noise")
os.makedirs(name=out_path, exist_ok=True)
out_file = os.path.join(out_path, "dataset.csv")
df = pd.DataFrame(data=noise_data, columns=NOISE_HEADING)
df.to_csv(path_or_buf=out_file, index=False)