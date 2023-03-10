import os
import random
import pandas as pd
from tqdm import tqdm
from math import floor
from typing import Tuple


LANGS = ["eng", "ita"]
DATASET_PATH = "./datasets/MIVIA_ISC"
NOISE_OUTPUT_PATH = os.path.join(DATASET_PATH, "annotations", "noise")
NOISE_ANNOTATIONS_FILE = os.path.join(NOISE_OUTPUT_PATH, "dataset.csv")
HEADING = ["path", "type", "subtype", "speaker", "label"]
NOISE_HEADING = ["path", "type", "subtype"]

TRAINING = 0.8
VALIDATION = 0.1
TEST = 0.1


def split_dataframe(df:pd.DataFrame, train_rate:float, valid_rate:float, test_rate:float) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """Splid a dataframe in training, validation and test set.

    Parameters
    ----------
    df: pandas.DataFrame
        Pandas dataframe containing the data to split
    train_rate: float
        Relative percentage of the sample to include in training set
    valid_rate: float
        Relative percentage of the sample to include in validation set
    test_rate: float
        Relative percentage of the sample to include in test set

    Returns
    -------
    pandas.DataFrame
        Pandas dataframe containing the training data
    pandas.DataFrame
        Pandas dataframe containing the validation data
    pandas.DataFrame
        Pandas dataframe containing the test data
    """
    n_sample = len(df)

    num_valid = floor(n_sample*valid_rate) if (floor(n_sample*valid_rate)>0 or n_sample<3) else 1
    num_test = floor(n_sample*test_rate) if (floor(n_sample*test_rate)>0 or n_sample<3) else 1
    num_train = n_sample - num_valid - num_test

    train_df = df.iloc[:num_train]
    valid_df = df.iloc[num_train:num_train+num_valid]
    test_df = df.iloc[num_train+num_valid:]

    return train_df, valid_df, test_df


def get_df_attribute_set(attribute_group, attribute_list:list, heading) -> pd.DataFrame:
    df_set = pd.DataFrame(columns=heading)

    for attribute in attribute_list:
        df_attribute = attribute_group.get_group(attribute)
        df_attribute = df_attribute.sample(frac=1)

        df_set = pd.concat(objs=[df_set, df_attribute])

    return df_set


############
# COMMANDS #
############
for lang in LANGS:
    annotation_file = os.path.join(DATASET_PATH, "annotations/{}/dataset.csv".format(lang))
    output_file = os.path.join(DATASET_PATH, "annotations/{}/".format(lang))

    ''' Read annotation file '''
    df = pd.read_csv(annotation_file, sep=',')

    ''' Shuffle the row of the dataframe '''
    df = df.sample(frac=1).reset_index()

    ''' Split dataset in training, validation and test set '''
    n_sample = len(df)
    train_dict = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "label": list()}
    valid_dict = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "label": list()}
    test_dict = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "label": list()}

    # BALANCING ON THE SPEAKER AMONG THE SETS
    speaker_group = df.groupby(df.speaker)
    speaker_iter = tqdm(speaker_group.groups)
    for speaker in speaker_iter:
        df_speaker = speaker_group.get_group(speaker).reset_index()
        
        for idx in df_speaker.index:
            temp = idx%10
            if temp == 0:
                test_dict["path"].append(df_speaker.iloc[idx]["path"])
                test_dict["type"].append(df_speaker.iloc[idx]["type"])
                test_dict["subtype"].append(df_speaker.iloc[idx]["subtype"])
                test_dict["speaker"].append(df_speaker.iloc[idx]["speaker"])
                test_dict["label"].append(df_speaker.iloc[idx]["label"])
            elif temp == 1:
                valid_dict["path"].append(df_speaker.iloc[idx]["path"])
                valid_dict["type"].append(df_speaker.iloc[idx]["type"])
                valid_dict["subtype"].append(df_speaker.iloc[idx]["subtype"])
                valid_dict["speaker"].append(df_speaker.iloc[idx]["speaker"])
                valid_dict["label"].append(df_speaker.iloc[idx]["label"])
            else:
                train_dict["path"].append(df_speaker.iloc[idx]["path"])
                train_dict["type"].append(df_speaker.iloc[idx]["type"])
                train_dict["subtype"].append(df_speaker.iloc[idx]["subtype"])
                train_dict["speaker"].append(df_speaker.iloc[idx]["speaker"])
                train_dict["label"].append(df_speaker.iloc[idx]["label"])
        
        speaker_iter.set_description("Analyzing <{}> dataset.".format(lang))

    """
    count = 0
    type_group = df.groupby(df.type)

    # BALANCING ON THE TYPE AMONG THE SETS
    for type in type_group.groups:
        df_type = type_group.get_group(type)
        df_type = df_type.sample(frac=1)
        # Balance on the type among the set
        df_type_sets = split_dataframe(df=df_type, train_rate=TRAINING, valid_rate=VALIDATION, test_rate=TEST)
        # print("The TYPE '{}' contains '{}' samples.\nTrain:\t{}\nValid:\t{}\nTest:\t{}".format(type, len(df_type), len(df_type_sets[0]), len(df_type_sets[1]), len(df_type_sets[2])))

        for df_type_set in df_type_sets:
            subtype_group = df_type_set.groupby(df.subtype)

            # BALANCING ON THE SUBTYPE AMONG THE SETS
            for subtype in subtype_group.groups:
                df_subtype = subtype_group.get_group(subtype)
                df_subtype = df_subtype.sample(frac=1)
                df_subtype_sets = split_dataframe(df=df_subtype, train_rate=TRAINING, valid_rate=VALIDATION, test_rate=TEST)
                # print("The TYPE {} SUBTYPE '{}' contains '{}' samples.\nTrain:\t{}\nValid:\t{}\nTest:\t{}".format(type, subtype, len(df_subtype), len(df_subtype_sets[0]), len(df_subtype_sets[1]), len(df_subtype_sets[2])))

                for df_subtype_set in df_subtype_sets:
                    speaker_group = df_subtype_set.groupby(df.speaker)

                    # BALANCING ON THE SPEAKER AMONG THE SETS
                    speaker_iter = tqdm(speaker_group.groups)
                    for speaker in speaker_iter:
                        df_speaker = speaker_group.get_group(speaker).reset_index()
                        
                        for idx in df_speaker.index:
                            temp = idx%10
                            if temp == 0:
                                test_df = pd.concat(objs=[test_df, df_speaker.iloc[idx]])
                            elif temp == 1:
                                valid_df = pd.concat(objs=[valid_df, df_speaker.iloc[idx]])
                            else:
                                train_df = pd.concat(objs=[train_df, df_speaker.iloc[idx]])
                        
                        speaker_iter.set_description("Analyzing '{}/{}'".format(type, subtype))
    """

# print("Counting: {}\nTraining: {}\nValidation: {}\nTesting: {}\n".format(type_subtype_group.count(), len(train_noise_df), len(valid_noise_df), len(test_noise_df)))
#################

    ''' WRITE CSV FILES '''
    #train_df = train_df.sample(frac=1)
    out_file = os.path.join(output_file, "training.csv")
    df = pd.DataFrame(data=train_dict, columns=HEADING).sample(frac=1)
    df.to_csv(path_or_buf=out_file, index=False)

    #valid_df = valid_df.sample(frac=1)
    out_file = os.path.join(output_file, "validation.csv")
    df = pd.DataFrame(data=valid_dict, columns=HEADING).sample(frac=1)
    df.to_csv(path_or_buf=out_file, index=False)

    #test_df = test_df.sample(frac=1)
    out_file = os.path.join(output_file, "testing.csv")
    df = pd.DataFrame(data=test_dict, columns=HEADING).sample(frac=1)
    df.to_csv(path_or_buf=out_file, index=False)


############
#   NOISE  #
############
''' Read NOISE annotation file '''
noise_df = pd.read_csv(NOISE_ANNOTATIONS_FILE, sep=',')

''' Shuffle the row of the NOISE dataframe '''
noise_df = noise_df.sample(frac=1)

''' Split NOISE dataset in training, validation and test set '''
n_sample = len(noise_df)
train_noise_df = pd.DataFrame(columns=NOISE_HEADING)
valid_noise_df = pd.DataFrame(columns=NOISE_HEADING)
test_noise_df = pd.DataFrame(columns=NOISE_HEADING)

### THIRD VERSION ###
type_subtype_group = noise_df.groupby([noise_df.type, noise_df.subtype])
train_list = [
    ("drill", "unknown"),
    ("crf_melfi", "move_arm"),
    ("crf_melfi", "unknown"),
    ("strumenti di lavoro (lavorazione metalli)", "air-ratchet (cricchetto ad aria)"),
    ("strumenti di lavoro (lavorazione metalli)", "grinder  (mola)"),
    ("strumenti di lavoro (lavorazione metalli)", "saldatrice"),
    ("strumenti di lavoro (lavorazione metalli)", "saw"),
    ("strumenti di lavoro (lavorazione metalli)", "compressore"),
    ("hammer", "unknown"),
    ("catena di lavoro", "unknown"),
    ("colpi ripetuti", "unknown"),
    ("generatore", "unknown"),
    ("fan", "unknown")
]
val_list = [
    ("crf_melfi", "move"),
    ("catena di lavoro robot", "unknown"),
    ("strumenti di lavoro (lavorazione metalli)", "macchina per incidere"),
    ("strumenti di lavoro (lavorazione metalli)", "raschiatura"),
    ("unknown", "unknown")
]
test_list = [
    ("crf_melfi", "human_talk"),
    ("strumenti di lavoro (lavorazione metalli)", "tornio")
]

train_noise_df = pd.concat(objs=[train_noise_df, get_df_attribute_set(attribute_group=type_subtype_group, attribute_list=train_list, heading=NOISE_HEADING)])
valid_noise_df = pd.concat(objs=[valid_noise_df, get_df_attribute_set(attribute_group=type_subtype_group, attribute_list=val_list, heading=NOISE_HEADING)])
test_noise_df = pd.concat(objs=[test_noise_df, get_df_attribute_set(attribute_group=type_subtype_group, attribute_list=test_list, heading=NOISE_HEADING)])

train_noise_df = train_noise_df.sample(frac=1)
out_file = os.path.join(NOISE_OUTPUT_PATH, "training.csv")
noise_df = pd.DataFrame(data=train_noise_df, columns=NOISE_HEADING)
noise_df.to_csv(path_or_buf=out_file, index=False)

valid_noise_df = valid_noise_df.sample(frac=1)
out_file = os.path.join(NOISE_OUTPUT_PATH, "validation.csv")
noise_df = pd.DataFrame(data=valid_noise_df, columns=NOISE_HEADING)
noise_df.to_csv(path_or_buf=out_file, index=False)

test_noise_df = test_noise_df.sample(frac=1)
out_file = os.path.join(NOISE_OUTPUT_PATH, "testing.csv")
noise_df = pd.DataFrame(data=test_noise_df, columns=NOISE_HEADING)
noise_df.to_csv(path_or_buf=out_file, index=False)