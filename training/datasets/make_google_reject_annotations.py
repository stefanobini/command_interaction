import os
import pandas
from tqdm import tqdm


IN_ANNOTATION_FILE = "testing_list.txt"
OUTPUT_ANNOTATION_FILE = "google_annotations.csv"
HEADING = ["path", "speaker", "label"]


data = {"path": list(), "speaker": list(), "label": list()}
with open(IN_ANNOTATION_FILE, 'r') as f_in:
    line_iter = tqdm(f_in)
    for line in line_iter:
        path = line.replace('\n', '')
        speaker = path.split('/')[1].split('_')[0]
        label = path.split('/')[0]

        data["path"].append(path)
        data["speaker"].append(speaker)
        data["label"].append(label)

        line_iter.set_description("Analyzing ", label)

df = pandas.DataFrame(data=data, columns=HEADING)

sort_speaker_df = df.groupby([speaker])["path"].count().reset_index(name="count").sort_values(["count"], ascending=False)

print(sort_speaker_df)