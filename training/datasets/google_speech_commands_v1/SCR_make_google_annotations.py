import os
import pandas
from tqdm import tqdm

DATASET_PATH = os.path.join("datasets", "google_speech_commands_v1")
IN_ANNOTATION_FILE_1 = os.path.join(DATASET_PATH, "testing_list.txt")
IN_ANNOTATION_FILE_2 = os.path.join(DATASET_PATH, "validation_list.txt")
OUTPUT_ANNOTATION_FILE = os.path.join(DATASET_PATH, "SCR_google_annotations_LANG.csv")
HEADING = ["path", "type", "subtype", "speaker", "command"]
N_REJECTS = 10000
MIN_N_SAMPLExSPEAKER = 10
REJECT_LABEL = 31
commands:dict={
    "eng":{"go":22, "stop":24},
    "ita":{"stop":24}
}
LANGs = ["eng", "ita"]

for lang in LANGs:
    data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}
    with open(IN_ANNOTATION_FILE_1, 'r') as f_in:
        for line in f_in:
            path = os.path.join("google_speech_commands_v1", line.replace('\n', ''))
            speaker = path.split('/')[1].split('_')[0]
            word = path.split('/')[1]
            command = commands[lang][word] if word in commands[lang] else REJECT_LABEL
            data["path"].append(path)
            data["type"].append("command")
            data["subtype"].append("google")
            data["speaker"].append(speaker)
            data["command"].append(command)

    with open(IN_ANNOTATION_FILE_2, 'r') as f_in:
        for line in f_in:
            path = os.path.join("google_speech_commands_v1", line.replace('\n', ''))
            speaker = path.split('/')[1].split('_')[0]
            word = path.split('/')[1]
            command = commands[lang][word] if word in commands[lang] else REJECT_LABEL
            data["path"].append(path)
            data["type"].append("command")
            data["subtype"].append("google")
            data["speaker"].append(speaker)
            data["command"].append(command)

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