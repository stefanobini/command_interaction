import os
import sys
import pandas
from tqdm import tqdm


LANGS = ["eng", "ita"]
IN_DATASET_FOLDER = "MIVIA_ISC"
OUT_DATASET_FOLDER = os.path.join("FELICE", "demo7_plus")
ANNOTATION_FILE = "dataset.csv"
COLUMNS = ["path", "type", "subtype", "speaker", "label"]
REJECT_PERCENTAGE = 0.5
OLD_CMD_LABEL_3_AND_7 = {
    2:0,
    3:1,
    23:2,
    20:3,
    6:4,
    7:5,
    16:6,
    17:7
}
OLD_CMD_LABEL_3 = {
    2:0,
    3:1,
    4:2,
    5:3
}
OLD_CMD_LABEL_7 = {
    23:0,
    20:1,
    6:2,
    7:3,
    16:4,
    17:5
}
OLD_CMD_LABEL_7_PLUS = {
    23:0,
    20:1,
    6:2,
    7:3,
    8:4,
    9:5,
    16:6,
    17:7
}
OLD_CMD_LABEL = OLD_CMD_LABEL_7_PLUS

for lang in LANGS:
    in_dataset_folder = os.path.join("datasets", IN_DATASET_FOLDER, "annotations", lang)
    os.makedirs(in_dataset_folder, exist_ok=True)
    in_dataset_path = os.path.join(in_dataset_folder, ANNOTATION_FILE)
    out_dataset_folder = os.path.join("datasets", OUT_DATASET_FOLDER, "annotations", lang)
    os.makedirs(out_dataset_folder, exist_ok=True)
    out_dataset_path = os.path.join(out_dataset_folder, ANNOTATION_FILE)
    in_df = pandas.read_csv(filepath_or_buffer=in_dataset_path, sep=',')
    out_dict = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "label": list()}

    df_iter = tqdm(in_df.index)
    for idx in df_iter:
        if in_df.iloc[idx]["label"] in OLD_CMD_LABEL:
            out_dict["path"].append(in_df.iloc[idx]["path"])
            out_dict["type"].append(in_df.iloc[idx]["type"])
            out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
            out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
            out_dict["label"].append(OLD_CMD_LABEL[in_df.iloc[idx]["label"]])
        else:
            out_dict["path"].append(in_df.iloc[idx]["path"])
            out_dict["type"].append("reject")
            out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
            out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
            out_dict["label"].append(len(OLD_CMD_LABEL))

        df_iter.set_description("Building <{}> annotation file.".format(lang))

    out_df = pandas.DataFrame(data=out_dict, columns=COLUMNS)

    df_group = out_df.groupby(out_df.type)
    reject_group = df_group.get_group("reject")
    samples = len(out_df)
    reject_samples = len(reject_group)
    command_samples = samples - reject_samples
    reject_samples_to_insert = command_samples * REJECT_PERCENTAGE / (1 - REJECT_PERCENTAGE)
    reject_samples_to_remove = reject_samples - reject_samples_to_insert
    #print(samples, command_samples, reject_samples, reject_samples_to_insert, reject_samples_to_remove)
    out_df.sample(frac=1, ignore_index=True)
    df_iter = tqdm(out_df.index)
    for index in df_iter:
        if reject_samples_to_remove > 0 and out_df["type"][index] == "reject":
            out_df.drop(index=index, inplace=True)
            reject_samples_to_remove -= 1
        df_iter.set_description("Remove reject samples from <{}> dataset to obtain the 'reject/command samples rate' of {}.".format(lang, REJECT_PERCENTAGE))

    out_df.to_csv(path_or_buf=out_dataset_path, index=False)