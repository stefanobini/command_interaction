"""
python3 datasets/utils/fix_speaker_id.py
"""

import os
import pandas
import numpy


LANGS = ["eng", "ita"]
DATASET_PATH = os.path.join("datasets", "MTL_experimentation_1", "annotations")
HEADING = ["path", "type", "subtype", "speaker", "command", "gender", "age"]
PEAKER_ID = ["id", "speaker", "gender", "age"] 


speaker_dict = {"id":list(), "speaker":list(), "gender":list(), "age":list()}
for lang in LANGS:
    annotation_fold = os.path.join(DATASET_PATH, lang)
    if not os.path.isdir(annotation_fold):
        os.makedirs(annotation_fold)
    in_annotation_file = os.path.join(annotation_fold, "dataset_no_speaker_id.csv")
    out_annotation_file = os.path.join(annotation_fold, "dataset.csv")
    speaker_file = os.path.join(annotation_fold, "speaker.csv")
    
    df = pandas.read_csv(in_annotation_file, sep=',')
    speakers = df["speaker"].unique()
    df_dict = df.to_dict()
    
    for idx in df.index:
        if len(numpy.where(speakers==df["speaker"][idx])) > 1:
            print(df["speaker"][idx])
        id = int(numpy.where(speakers==df["speaker"][idx])[0])
        df_dict["speaker"][idx] = id
    
    for id in range(len(speakers)):
        speaker_dict["id"].append(id)
        speaker_dict["speaker"].append(speakers[id])
        i = df[df["speaker"] == speakers[id]].index[0]
        speaker_dict["gender"].append(df["gender"][i])
        speaker_dict["age"].append(df["age"][i])
    
    speaker_df = pandas.DataFrame(speaker_dict)
    speaker_df.to_csv(path_or_buf=speaker_file, index=False)
    df = pandas.DataFrame(data=df_dict, columns=HEADING)
    df.to_csv(out_annotation_file, index=False)