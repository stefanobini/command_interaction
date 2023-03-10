import os
import pandas
from tqdm import tqdm


IN_DATASET_PATH = "./MIVIA_ISC"
OUTPUT_DATASET_PATH = "./MTL_experimentation"
HEADING = ["path", "type", "subtype", "speaker", "label"]


for lang in ["eng", "ita"]:
    in_annotation_file = os.path.join(IN_DATASET_PATH, "annotations", lang, "dataset.csv")
    output_annotation_file = os.path.join(OUTPUT_DATASET_PATH, "annotations", lang, "dataset.csv")
    MIN_N_SAMPLExSPEAKER = 10

    in_df = pandas.read_csv(in_annotation_file, sep=',')
    # in_cmd_df = in_df.loc[in_df["type"] == "command"]

    sort_speaker_df = in_df.groupby(["speaker"])["path"].count().reset_index(name="count").sort_values(["count"], ascending=False).reset_index()

    speakers = list()
    accumulator = 0
    for idx in sort_speaker_df.index:
        accumulator += 1
        speaker = sort_speaker_df["speaker"][idx]
        n_speaker = sort_speaker_df["count"][idx]
        speakers.append(speaker)
        if n_speaker < MIN_N_SAMPLExSPEAKER:
            break

    #print(sort_speaker_df.iloc[-20:])
    print("{}/{} speakers have more than {} samples.".format(accumulator, len(sort_speaker_df), MIN_N_SAMPLExSPEAKER))

    out_df = in_df.loc[in_df["speaker"].isin(speakers)]
    out_df.to_csv(path_or_buf=output_annotation_file, index=False)