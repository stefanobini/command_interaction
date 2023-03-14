import os
import pandas
from tqdm import tqdm


IN_DATASET_PATH = "./datasets/MIVIA_ISC"
OUTPUT_DATASET_PATH = "./datasets/MTL_experimentation"
LANGS = ["eng", "ita"]
HEADING = ["path", "type", "subtype", "speaker", "label"]


for lang in LANGS:
    in_annotation_file = os.path.join(IN_DATASET_PATH, "annotations", lang, "dataset.csv")
    output_annotation_folder = os.path.join(OUTPUT_DATASET_PATH, "annotations", lang)
    if not os.path.isdir(output_annotation_folder):
            os.makedirs(output_annotation_folder)
    output_annotation_file = os.path.join(output_annotation_folder, "dataset_no_speaker_id.csv")
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
        if n_speaker < MIN_N_SAMPLExSPEAKER:
            break
        speakers.append(speaker)

    # print(sort_speaker_df.iloc[-10:])
    print("{}/{} speakers have more than {} samples.".format(accumulator, len(sort_speaker_df), MIN_N_SAMPLExSPEAKER))

    out_df = in_df.loc[in_df["speaker"].isin(speakers)]
    # print(out_df.groupby(["speaker"])["path"].count().reset_index(name="count").sort_values(["count"], ascending=False).reset_index().iloc[-10:])
    out_df.to_csv(path_or_buf=output_annotation_file, index=False)