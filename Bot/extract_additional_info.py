import os
import json
import pandas
from tqdm import tqdm


DATASET_PATH = "saves"
INFO_FILENAME = "info.json"
OUT_PATH = os.path.join("..", "training", "datasets", "additional_info.csv")
HEADING = ["speaker", "gender", "age", "eng_cmds", "ita_cmds"]
LANGS = ["eng", "ita"]


"""TELEGRAM SAMPLES"""
database = {"speaker":list(), "gender":list(), "age":list(), "eng_cmds":list(), "ita_cmds":list()}
speaker_iter = tqdm(os.listdir(DATASET_PATH))
for speaker in speaker_iter:
    speaker_info = dict()
    speaker_path = os.path.join(DATASET_PATH, speaker)
    info_filename_path = os.path.join(speaker_path, INFO_FILENAME)
    langs = os.listdir(speaker_path)
    for lang in LANGS:
        lang_path = os.path.join(speaker_path, lang)
        if os.path.exists(lang_path):
            filenames = os.listdir(lang_path)
            n_cmds = 0
            for filename in filenames:
                n_cmds += 1 if filename != "desktop.ini" else 0
            database["{}_cmds".format(lang)].append(n_cmds)
        else:
            database["{}_cmds".format(lang)].append(0)
    with open(info_filename_path) as f:
        info = json.load(f)
        database["speaker"].append(speaker)
        database["gender"].append(info["gender"])
        database["age"].append(info["age"])
    speaker_iter.set_description("Analyzing dataset")

df = pandas.DataFrame(data=database, columns=HEADING)
df.to_csv(path_or_buf=OUT_PATH, index=False)