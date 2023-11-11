"""
python3 datasets/utils/FELICE_make_annotations.py

CREATE THE DATASET CSV FILES, one in english and one in italian language.

The dataset attributes are the followings.
    path:       path of the audio file
    type:       {real, sythetic, reject}
    subtype:    for real {smartphone, respeaker, ...}
                for synthetic {azure, google, naturalreaders, nemo, polly, vocalware}
                for reject {google_speech, mozilla_common_voice}
    speaker:    speaker ID
    label:    label of teh command pronunced inside the sample
"""

import os
import pandas as pd
from tqdm import tqdm
import pandas
from FELICE_cmd_map import N_COMMAND_SAMPLES, DEMO_3, DEMO_7


DEMO = "7"

DATASET_NAME = os.path.join("FELICE", "demo"+DEMO)
IN_PATH = os.path.join("datasets", DATASET_NAME)
OUT_DATASET = os.path.join("datasets", DATASET_NAME)
OUT_PATH = os.path.join(OUT_DATASET, "annotations")
OUT_ANNOTATION_FILENAME = "dataset.csv"
REJECT_DATASETS = ["google_speech_commands_v1", "mozilla_common_voices"]#, DATASET_NAME]
REJECT_NAMES = ["google", "mozilla"]#, "telegram"]
REJECT_ANNOTATION_PATHS = [os.path.join(IN_PATH, "{}_annotations_LANG.csv".format(dataset)) for dataset in REJECT_NAMES]
LANGS = ["eng", "ita"]
NOISE_SAMPLES = N_COMMAND_SAMPLES/3
HEADING = ["path", "type", "subtype", "speaker", "command"]
NOISE_HEADING = ["path", "type", "subtype"]

if DEMO == str(3):
    COMMANDS_DICT = DEMO_3
elif DEMO == str(7):
    COMMANDS_DICT = DEMO_7
else:
    print("Demo <{}> not supported.".format(DEMO))
COMMAND_LIST = list(COMMANDS_DICT.values())


cmd_path = "commands"
rjt_path = "rejects"
noise_path = "noises"


def add_command_samples(data, lang_path, speaker, samples):
    type = "command"
    for sample in samples:
        sample_path = os.path.join(lang_path, sample).replace("./", "")
        subtype = "telegram" if len(sample.split('_'))==2 else "google"
        command = int(sample.split('.')[0].split('_')[1]) #if sample.split("_")[1] not in LANGS else int(sample.split("_")[2].split(".")[0])
        remapped_command = COMMANDS_DICT[command]
        data["path"].append(sample_path)
        data["type"].append(type)
        data["subtype"].append(subtype)
        data["speaker"].append(speaker)
        data["command"].append(remapped_command)
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
        data["command"].append(len(COMMANDS_DICT))
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

''' REJECT SAMPLES '''
for lang in LANGS:
    for i in range(len(REJECT_ANNOTATION_PATHS)):
        data[lang] = add_reject_samples(path=REJECT_ANNOTATION_PATHS[i].replace("LANG", "{}".format(lang.upper())), lang=lang, data=data[lang])

''' NOISE SAMPLES'''
in_noise_folder = os.path.join("datasets", DATASET_NAME, "annotations", "noise")
in_noise_path = os.path.join(in_noise_folder, "dataset.csv")
in_noise_df = pandas.read_csv(filepath_or_buffer=in_noise_path, sep=',')
in_noise_df = in_noise_df.sample(frac=1)
for lang in LANGS:
    add_noises = 0
    for idx in in_noise_df.index:
        data[lang]["path"].append(in_noise_df.iloc[idx]["path"])
        data[lang]["type"].append("reject")
        data[lang]["subtype"].append(in_noise_df.iloc[idx]["subtype"])
        data[lang]["speaker"].append("unknown")
        data[lang]["command"].append(len(COMMANDS_DICT))
        add_noises += 1
        if add_noises > NOISE_SAMPLES:
            break

''' WRITE CSV FILE '''
for lang in LANGS:
    out_path = os.path.join(OUT_PATH, lang)
    os.makedirs(name=out_path, exist_ok=True)
    out_file = os.path.join(out_path, OUT_ANNOTATION_FILENAME)
    out_df = pd.DataFrame(data=data[lang], columns=HEADING)
    out_df.to_csv(path_or_buf=out_file, index=False)