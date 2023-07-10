import os
import pandas
from datasets.scripts.commands import COMMANDS
from datasets.scripts.intents import INTENTS, EXPLICIT_INTENTS, IMPLICIT_INTENTS


EXPERIEMNTATION = "MSI"
INPUT_DATASET = os.path.join("datasets", "mozilla_common_voices")
OUTPUT_ANNOTATION_FILE = os.path.join(INPUT_DATASET, "{}_mozilla_annotations_LANG.csv".format(EXPERIEMNTATION))
LANGs = ["eng", "esp", "ita"]
if EXPERIEMNTATION == "SCR":
    HEADING = ["path", "type", "subtype", "speaker", "command"]
elif EXPERIEMNTATION == "MSI":
    HEADING = ["path", "type", "subtype", "speaker", "intent", "explicit", "implicit"]
N_REJECTS = 10000
MIN_N_SAMPLExSPEAKER = 10


for lang in LANGs:
    if EXPERIEMNTATION == "SCR":
        data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}
    elif EXPERIEMNTATION == "MSI":
        data = {"path":list(), "type":list(), "subtype":list(), "speaker":list(), "intent":list(), "explicit":list(), "implicit":list()}
    lang_path = os.path.join(INPUT_DATASET, lang)
    for sample in os.listdir(lang_path):
        data["path"].append(os.path.join("mozilla_common_voices", lang, sample))
        data["type"].append("reject")
        data["subtype"].append("mozilla")
        data["speaker"].append(sample.split('_')[-1].split('.')[0])
        if EXPERIEMNTATION == "SCR":
            data["command"].append(len(COMMANDS)-1)
        elif EXPERIEMNTATION == "MSI":
            data["intent"].append(len(INTENTS)-1)
            data["explicit"].append(len(EXPLICIT_INTENTS[lang])-1)
            data["implicit"].append(0)

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