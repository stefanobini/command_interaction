import os
import pandas
import numpy


LANGS = ["eng", "ita"]
DATASET_PATH = "./datasets/MTL_scr_srid/annotations"
HEADING = ["path", "type", "subtype", "speaker", "label"]

speaker_dict = dict()
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
        df_dict["speaker"][idx] = int(numpy.where(speakers==df["speaker"][idx])[0])
    
    speaker_df = pandas.DataFrame(speakers)
    speaker_df.to_csv(path_or_buf=speaker_file, index=True)
    df = pandas.DataFrame(data=df_dict, columns=HEADING)
    df.to_csv(out_annotation_file, index=False)