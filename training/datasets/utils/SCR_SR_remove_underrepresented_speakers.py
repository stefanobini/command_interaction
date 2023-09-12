import os
import pandas
from tqdm import tqdm


LANGS = ["eng", "ita"]

IN_DATASET_PATH = "./datasets/MIVIA_ISC_v2"
OUTPUT_DATASET_PATH = "./datasets/MTL_experimentation_1"
os.makedirs(OUTPUT_DATASET_PATH, exist_ok=True)

HEADING = ["path", "type", "subtype", "speaker", "command"]

MIN_N_SAMPLExSPEAKER = 25


for lang in LANGS:
    in_annotation_file = os.path.join(IN_DATASET_PATH, "annotations", lang, "dataset.csv")
    output_annotation_folder = os.path.join(OUTPUT_DATASET_PATH, "annotations", lang)
    if not os.path.isdir(output_annotation_folder):
            os.makedirs(output_annotation_folder)

    in_df = pandas.read_csv(in_annotation_file, sep=',')

    sort_speaker_df = in_df.groupby(["speaker"])["path"].count().reset_index(name="count").sort_values(["count"], ascending=False).reset_index()

    speakers = list()
    for idx in sort_speaker_df.index:
        speaker = sort_speaker_df["speaker"][idx]
        n_sample = sort_speaker_df["count"][idx]
        if n_sample > MIN_N_SAMPLExSPEAKER:
            speakers.append(speaker)

    # print(sort_speaker_df.iloc[-10:])
    print("LANGUAGE:\t'{}'\nSelected speakers:\t{}\nRemoved speakers:\t{}\n".format(lang, len(speakers), len(sort_speaker_df)-len(speakers)))

    output_annotation_file = os.path.join(output_annotation_folder, "dataset_no_speaker_id.csv")
    out_df = in_df.loc[in_df["speaker"].isin(speakers)]
    # print(out_df.groupby(["speaker"])["path"].count().reset_index(name="count").sort_values(["count"], ascending=False).reset_index().iloc[-10:])
    out_df.to_csv(path_or_buf=output_annotation_file, index=False)

    output_speaker_count = os.path.join(output_annotation_folder, "dataset_speakers_count.csv")
    # print(out_df.groupby(["speaker"])["path"].count().reset_index(name="count").sort_values(["count"], ascending=False).reset_index().iloc[-10:])
    sort_speaker_df.to_csv(path_or_buf=output_speaker_count, index=False)