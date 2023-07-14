import os
import pandas
from tqdm import tqdm

DATASET_PATH = os.path.join("datasets", "google_speech_commands_v1")
IN_ANNOTATION_FILE_1 = os.path.join(DATASET_PATH, "testing_list.txt")
IN_ANNOTATION_FILE_2 = os.path.join(DATASET_PATH, "validation_list.txt")
OUTPUT_ANNOTATION_FILE = os.path.join(DATASET_PATH, "SCR_google_annotations.csv")
HEADING = ["path", "type", "subtype", "speaker", "command"]
N_REJECTS = 10000
MIN_N_SAMPLExSPEAKER = 10
REJECT_LABEL = 31


data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}
with open(IN_ANNOTATION_FILE_1, 'r') as f_in:
    for line in f_in:
        path = os.path.join("google_speech_commands_v1", line.replace('\n', ''))
        speaker = path.split('/')[1].split('_')[0]
        data["path"].append(path)
        data["type"].append("reject")
        data["subtype"].append("google_speech")
        data["speaker"].append(speaker)
        data["command"].append(REJECT_LABEL)

with open(IN_ANNOTATION_FILE_2, 'r') as f_in:
    for line in f_in:
        path = os.path.join("google_speech_commands_v1", line.replace('\n', ''))
        speaker = path.split('/')[1].split('_')[0]
        data["path"].append(path)
        data["type"].append("reject")
        data["subtype"].append("google_speech")
        data["speaker"].append(speaker)
        data["command"].append(REJECT_LABEL)

df = pandas.DataFrame(data=data, columns=HEADING)
df.to_csv(path_or_buf=OUTPUT_ANNOTATION_FILE, index=False)
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