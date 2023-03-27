import os
import pandas
from tqdm import tqdm


IN_DATASET_PATH = "./datasets/MIVIA_ISC"
OUTPUT_DATASET_PATH = "./datasets/MTL_experimentation"
LANGS = ["eng", "ita"]
HEADING = ["path", "type", "subtype", "speaker", "label"]
MIN_N_SAMPLExSPEAKER = 11
TEST_SPEAKER = 100
TEST_MIN_N_SAMPLExSPEAKER = 20
TEST_MAX_N_SAMPLExSPEAKER = {"eng": 31, "ita": 29}


for lang in LANGS:
    in_annotation_file = os.path.join(IN_DATASET_PATH, "annotations", lang, "dataset.csv")
    output_annotation_folder = os.path.join(OUTPUT_DATASET_PATH, "annotations", lang)
    if not os.path.isdir(output_annotation_folder):
            os.makedirs(output_annotation_folder)

    in_df = pandas.read_csv(in_annotation_file, sep=',')
    # in_cmd_df = in_df.loc[in_df["type"] == "command"

    sort_speaker_df = in_df.groupby(["speaker"])["path"].count().reset_index(name="count").sort_values(["count"], ascending=False).reset_index()

    train_speakers = list()
    test_speakers = list()
    for idx in sort_speaker_df.index:
        speaker = sort_speaker_df["speaker"][idx]
        n_sample = sort_speaker_df["count"][idx]
        if n_sample > MIN_N_SAMPLExSPEAKER:
            if (len(test_speakers) < TEST_SPEAKER) and (n_sample > TEST_MIN_N_SAMPLExSPEAKER) and (n_sample < TEST_MAX_N_SAMPLExSPEAKER[lang]):
                test_speakers.append(speaker)
            else:
                train_speakers.append(speaker)

    # print(sort_speaker_df.iloc[-10:])
    print("LANGUAGE: '{}'\nTraining speaker:\t{}\nTest speaker:\t{}\nRemoved speakers:\t{}\n".format(lang, len(train_speakers), len(test_speakers), len(sort_speaker_df)-len(train_speakers)-len(test_speakers)))

    output_annotation_file = os.path.join(output_annotation_folder, "training.csv")
    out_df = in_df.loc[in_df["speaker"].isin(train_speakers)]
    # print(out_df.groupby(["speaker"])["path"].count().reset_index(name="count").sort_values(["count"], ascending=False).reset_index().iloc[-10:])
    out_df.to_csv(path_or_buf=output_annotation_file, index=False)

    output_annotation_file = os.path.join(output_annotation_folder, "testing.csv")
    out_df = in_df.loc[in_df["speaker"].isin(test_speakers)]
    # print(out_df.groupby(["speaker"])["path"].count().reset_index(name="count").sort_values(["count"], ascending=False).reset_index().iloc[-10:])
    out_df.to_csv(path_or_buf=output_annotation_file, index=False)