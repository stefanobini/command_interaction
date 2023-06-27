import os
import pandas as pd
from tqdm import tqdm
from math import floor
from typing import Tuple


LANGS = ["eng", "ita"]

DATASET_PATH = "./datasets/MTL_scr_srid"
if not os.path.isdir(DATASET_PATH):
        os.makedirs(DATASET_PATH)

N_SAMPLExSPEAKER = [1, 3, 5, 10, 15, 20]
HEADING = ["path", "type", "subtype", "speaker", "command"]


for lang in LANGS:
    annotation_file = os.path.join(DATASET_PATH, "annotations", lang, "dataset.csv")
    output_path = os.path.join(DATASET_PATH, "annotations", lang)
    if not os.path.isdir(output_path):
        os.makedirs(output_path)
    
    ''' Split dataset in training and test set '''
    n_sample_iter = tqdm(N_SAMPLExSPEAKER)
    for n_sampleXspeaker in n_sample_iter:
        
        ''' Read annotation file '''
        df = pd.read_csv(annotation_file, sep=',')

        ''' Shuffle the row of the dataframe '''
        df = df.sample(frac=1).reset_index()

        train_dict = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}
        test_dict = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}

        # BALANCING ON THE SPEAKER AMONG THE SETS
        speaker_group = df.groupby(df.speaker)
        for speaker in speaker_group.groups:
            df_speaker = speaker_group.get_group(speaker).reset_index()
            
            sample = 0
            for idx in df_speaker.index:
                if sample < n_sampleXspeaker:
                    train_dict["path"].append(df_speaker.iloc[idx]["path"])
                    train_dict["type"].append(df_speaker.iloc[idx]["type"])
                    train_dict["subtype"].append(df_speaker.iloc[idx]["subtype"])
                    train_dict["speaker"].append(df_speaker.iloc[idx]["speaker"])
                    train_dict["command"].append(df_speaker.iloc[idx]["command"])
                else:
                    test_dict["path"].append(df_speaker.iloc[idx]["path"])
                    test_dict["type"].append(df_speaker.iloc[idx]["type"])
                    test_dict["subtype"].append(df_speaker.iloc[idx]["subtype"])
                    test_dict["speaker"].append(df_speaker.iloc[idx]["speaker"])
                    test_dict["command"].append(df_speaker.iloc[idx]["command"])
                sample += 1
            
        n_sample_iter.set_description("Building <{}> dataset with <{}> samples per speaker.".format(lang, n_sampleXspeaker))

        ''' WRITE CSV FILES '''
        #train_df = train_df.sample(frac=1)
        out_file = os.path.join(output_path, "training_{}_samples.csv".format(n_sampleXspeaker))
        df = pd.DataFrame(data=train_dict, columns=HEADING).sample(frac=1)
        df.to_csv(path_or_buf=out_file, index=False)

        #valid_df = valid_df.sample(frac=1)
        out_file = os.path.join(output_path, "testing_{}_samples.csv".format(n_sampleXspeaker))
        df = pd.DataFrame(data=test_dict, columns=HEADING).sample(frac=1)
        df.to_csv(path_or_buf=out_file, index=False)