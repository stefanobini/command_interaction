import os
import pandas
from commands import COMMANDS
from intents import INTENTS, EXPLICIT_INTENTS, IMPLICIT_INTENTS, REDUCED_INTENTS_DICT


EXPERIMENTATION = "MSI_exp0"
INPUT_DATASET = os.path.join("datasets", EXPERIMENTATION, "rejects")
OUTPUT_ANNOTATION_FILE = os.path.join("datasets", EXPERIMENTATION, "{}_annotations_LANG.csv".format(EXPERIMENTATION))
HEADING = ["path", "type", "subtype", "speaker", "command"]
LANGs = ["eng", "esp", "ita"]
if EXPERIMENTATION == "SCR":
    n_intents = len(COMMANDS)-1
elif EXPERIMENTATION == "MSI_exp0":
    n_intents = len(REDUCED_INTENTS_DICT)-1
N_REJECTS = 10000
MIN_N_SAMPLExSPEAKER = 10


for lang in LANGs:
    data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}
    lang_path = os.path.join(INPUT_DATASET, lang)
    for sample in os.listdir(lang_path):
        data["path"].append(os.path.join(EXPERIMENTATION, lang, sample))
        data["type"].append("reject")
        data["subtype"].append("telegram")
        data["speaker"].append(sample.split('_')[1])
        data["command"].append(n_intents)
    df = pandas.DataFrame(data=data, columns=HEADING)
    df.to_csv(path_or_buf=OUTPUT_ANNOTATION_FILE.replace("LANG", lang.upper()), index=False)
"""
sort_speaker_df = df.groupby(["speaker"])["path"].count().reset_index(name="count").sort_values(["count"], ascending=False).reset_index()

speakers = list()
accumulator = 0
for idx in sort_speaker_df.index:
    speaker = sort_speaker_df["speaker"][idx]
    n_speaker = sort_speaker_df["count"][idx]
    speakers.append(speaker)
    accumulator += n_speaker
    if accumulator > N_REJECTS or n_speaker < MIN_N_SAMPLExSPEAKER:
        break

out_df = df.loc[df["speaker"].isin(speakers)]

out_df.to_csv(path_or_buf=OUTPUT_ANNOTATION_FILE, index=False)
"""