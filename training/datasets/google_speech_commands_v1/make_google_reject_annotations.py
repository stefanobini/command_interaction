import os
import pandas
from tqdm import tqdm


IN_ANNOTATION_FILE_1 = "testing_list.txt"
IN_ANNOTATION_FILE_2 = "validation_list.txt"
OUTPUT_ANNOTATION_FILE = "google_annotations.csv"
HEADING = ["path", "type", "subtype", "speaker", "label"]
N_REJECTS = 12000
MIN_N_SAMPLExSPEAKER = 10


data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "label": list()}
with open(IN_ANNOTATION_FILE_1, 'r') as f_in:
    line_iter = tqdm(f_in)
    for line in line_iter:
        path = line.replace('\n', '')
        speaker = path.split('/')[1].split('_')[0]
        label = path.split('/')[0]

        data["path"].append(path)
        data["type"].append("reject")
        data["subtype"].append("google_speech")
        data["speaker"].append(speaker)
        data["label"].append(label)

with open(IN_ANNOTATION_FILE_2, 'r') as f_in:
    line_iter = tqdm(f_in)
    for line in line_iter:
        path = line.replace('\n', '')
        speaker = path.split('/')[1].split('_')[0]
        label = path.split('/')[0]

        data["path"].append(path)
        data["type"].append("reject")
        data["subtype"].append("google_speech")
        data["speaker"].append(speaker)
        data["label"].append(label)

df = pandas.DataFrame(data=data, columns=HEADING)

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