"""
python3 datasets/utils/fix_synthetic_google_speakers.py 
"""

import os
import pandas
from tqdm import tqdm
from typing import List


DATASET_NAME = "MIVIA_ISC_v2"
DATASET_ANNOTATION_PATH = os.path.join("datasets", DATASET_NAME, "annotations")
LANGS = ["eng", "ita"]
HEADING = ["path", "type", "subtype", "speaker", "command"]


for lang in LANGS:
    out_df = {"path":list(), "type":list(), "subtype":list(), "speaker":list(), "command":list()}
    in_annotation_path = os.path.join(DATASET_ANNOTATION_PATH, lang, "old_dataset.csv")
    out_annotation_path = os.path.join(DATASET_ANNOTATION_PATH, lang, "fixed_dataset.csv")
    in_df = pandas.read_csv(in_annotation_path, sep=',', dtype=str)
    row_iter = tqdm(in_df.index)
    for i in row_iter:
        speaker = in_df["speaker"][i]
        # Handle Google synthetic samples
        if in_df["subtype"][i] in ["google_neural", "google_standard"]:
            if "/eng/" in in_df["path"][i]:
                speaker = "google_male"
            elif "/ita/" in in_df["path"][i]:
                gender = "google_female"
        # Handle Nemo synthetic samples
        elif in_df["subtype"][i] == "nemo_neural":
            speaker = "nemo"
        # Handle VocalWare synthetic samples
        elif in_df["subtype"][i] == "vocalware_neural":
            if "ita/engine2_voice5" in in_df["path"][i] or "ita/engine2_voice7" in in_df["path"][i]:
                continue
        out_df["path"].append(in_df["path"][i])
        out_df["type"].append(in_df["type"][i])
        out_df["subtype"].append(in_df["subtype"][i])
        out_df["speaker"].append(speaker)
        out_df["command"].append(in_df["command"][i])

    out_df = pandas.DataFrame(data=out_df, columns=HEADING)
    out_df = out_df.sample(frac=1)
    out_df.to_csv(path_or_buf=out_annotation_path, index=False)