import os
import pandas


LANGS = ["eng", "ita"]
INPUT_DATASET_PATH = "./datasets/MTL_experimentation_0"
OUTPUT_SPEAKER_ID_FILE = INPUT_DATASET_PATH

speaker_dict = dict()
for lang in LANGS:
    in_annotation_file = os.path.join(INPUT_DATASET_PATH, "annotations", lang, "dataset.csv")
    output_annotation_file = os.path.join(OUTPUT_SPEAKER_ID_FILE, "annotations", lang, "speaker.csv")
    in_df = pandas.read_csv(in_annotation_file, sep=',')
    speakers = in_df["speaker"].unique()
    speaker_df = pandas.DataFrame(speakers)
    speaker_df.to_csv(path_or_buf=output_annotation_file, index=True)        