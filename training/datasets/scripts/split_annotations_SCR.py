import os
import random
import pandas as pd
from tqdm import tqdm
from math import floor
from typing import Tuple


HEADING = ["path", "type", "subtype", "speaker", "label"]
NOISE_HEADING = ["path", "type", "subtype"]
LANG = "eng"

DATASET_PATH = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1"
ANNOTATIONS_FILE = os.path.join(DATASET_PATH, "annotations/{}/dataset.csv".format(LANG))
OUTPUT_PATH = os.path.join(DATASET_PATH, "annotations/{}/".format(LANG))
NOISE_ANNOTATIONS_FILE = os.path.join(DATASET_PATH, "annotations", "noise", "dataset.csv")
NOISE_OUTPUT_PATH = os.path.join(DATASET_PATH, "annotations", "noise")

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


''' Read annotation file '''
df = pd.read_csv(ANNOTATIONS_FILE, sep=',')

''' Shuffle the row of the dataframe '''
df = df.sample(frac=1)

''' Split dataset in training, validation and test set '''
n_sample = len(df)
train_df = pd.DataFrame(columns=HEADING)
valid_df = pd.DataFrame(columns=HEADING)
test_df = pd.DataFrame(columns=HEADING)

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

        # BALANCING ON THE SUPTYPE AMONG THE SETS
        for subtype in subtype_group.groups:
            df_subtype = subtype_group.get_group(subtype)
            df_subtype = df_subtype.sample(frac=1)
            df_subtype_sets = split_dataframe(df=df_subtype, train_rate=TRAINING, valid_rate=VALIDATION, test_rate=TEST)
            # print("The TYPE {} SUBTYPE '{}' contains '{}' samples.\nTrain:\t{}\nValid:\t{}\nTest:\t{}".format(type, subtype, len(df_subtype), len(df_subtype_sets[0]), len(df_subtype_sets[1]), len(df_subtype_sets[2])))

            for df_subtype_set in df_subtype_sets:
                speaker_group = df_subtype_set.groupby(df.speaker)
                
                if type != "reject":
                    n_speaker = len(speaker_group.groups)
                    num_valid_speaker = floor(n_speaker*VALIDATION) if (floor(n_speaker*VALIDATION)>0 or n_speaker<3) else 1
                    num_test_speaker = floor(n_speaker*TEST) if (floor(n_speaker*TEST)>0 or n_speaker<3) else 1
                    num_train_speaker = n_speaker - num_valid_speaker - num_test_speaker

                    train_speakers = list(speaker_group.groups.keys())[:num_train_speaker]
                    valid_speakers = list(speaker_group.groups.keys())[num_train_speaker:num_train_speaker+num_valid_speaker]
                    test_speakers = list(speaker_group.groups.keys())[num_train_speaker+num_valid_speaker:]

                    train_df = pd.concat(objs=[train_df, get_df_attribute_set(attribute_group=speaker_group, attribute_list=train_speakers, heading=HEADING)])
                    valid_df = pd.concat(objs=[valid_df, get_df_attribute_set(attribute_group=speaker_group, attribute_list=valid_speakers, heading=HEADING)])
                    test_df = pd.concat(objs=[test_df, get_df_attribute_set(attribute_group=speaker_group, attribute_list=test_speakers, heading=HEADING)])
                else:
                    # BALANCING ON THE SPEAKER AMONG THE SETS
                    for speaker in speaker_group.groups:
                        df_speaker = speaker_group.get_group(speaker)
                        df_speaker = df_speaker.sample(frac=1)
                        df_speaker_sets = split_dataframe(df=df_speaker, train_rate=TRAINING, valid_rate=VALIDATION, test_rate=TEST)

                        train_df = pd.concat(objs=[train_df, df_speaker_sets[0]])
                        valid_df = pd.concat(objs=[valid_df, df_speaker_sets[1]])
                        test_df = pd.concat(objs=[test_df, df_speaker_sets[2]])

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

# print("Counting: {}\nTraining: {}\nValidation: {}\nTesting: {}\n".format(type_subtype_group.count(), len(train_noise_df), len(valid_noise_df), len(test_noise_df)))
#################

'''
count = 0
type_group = noise_df.groupby(noise_df.type)

# BALANCING ON THE TYPE AMONG THE SETS
for type in type_group.groups:
    df_type = type_group.get_group(type)
    df_type = df_type.sample(frac=1)
    # Balance on the type among the set
    df_type_sets = split_dataframe(df=df_type, train_rate=TRAINING, valid_rate=VALIDATION, test_rate=TEST)
    # print("The TYPE '{}' contains '{}' samples.\nTrain:\t{}\nValid:\t{}\nTest:\t{}".format(type, len(df_type), len(df_type_sets[0]), len(df_type_sets[1]), len(df_type_sets[2])))

    for df_type_set in df_type_sets:
        subtype_group = df_type_set.groupby(noise_df.subtype)

        # BALANCING ON THE SUPTYPE AMONG THE SETS
        for subtype in subtype_group.groups:
            
            #### SECOND VERSION
            n_subtype = len(subtype_group.groups)
            num_valid_subtype = floor(n_subtype*VALIDATION) if (floor(n_subtype*VALIDATION)>0 or n_subtype<3) else 1
            num_test_subtype = floor(n_subtype*TEST) if (floor(n_subtype*TEST)>0 or n_subtype<3) else 1
            num_train_subtype = n_subtype - num_valid_subtype - num_test_subtype

            train_subtype = list(subtype_group.groups.keys())[:num_train_subtype]
            valid_subtype = list(subtype_group.groups.keys())[num_train_subtype:num_train_subtype+num_valid_subtype]
            test_subtype = list(subtype_group.groups.keys())[num_train_subtype+num_valid_subtype:]

            print("TYPE: {}\nTraining: {}\nValidation: {}\nTesting: {}\n".format(type, train_subtype, valid_subtype, test_subtype))

            train_noise_df = pd.concat(objs=[train_noise_df, get_df_attribute_set(attribute_group=subtype_group, attribute_list=train_subtype, heading=NOISE_HEADING)])
            valid_noise_df = pd.concat(objs=[valid_noise_df, get_df_attribute_set(attribute_group=subtype_group, attribute_list=valid_subtype, heading=NOISE_HEADING)])
            test_noise_df = pd.concat(objs=[test_noise_df, get_df_attribute_set(attribute_group=subtype_group, attribute_list=test_subtype, heading=NOISE_HEADING)])
            
            #### FIRST VERION
            df_subtype = subtype_group.get_group(subtype)
            df_subtype = df_subtype.sample(frac=1)
            df_subtype_sets = split_dataframe(df=df_subtype, train_rate=TRAINING, valid_rate=VALIDATION, test_rate=TEST)

            train_noise_df = pd.concat(objs=[train_noise_df, df_subtype_sets[0]])
            valid_noise_df = pd.concat(objs=[valid_noise_df, df_subtype_sets[1]])
            test_noise_df = pd.concat(objs=[test_noise_df, df_subtype_sets[2]])
'''

''' WRITE CSV FILES '''
train_df = train_df.sample(frac=1)
out_file = os.path.join(OUTPUT_PATH, "training.csv")
df = pd.DataFrame(data=train_df, columns=HEADING)
df.to_csv(path_or_buf=out_file, index=False)

train_noise_df = train_noise_df.sample(frac=1)
out_file = os.path.join(NOISE_OUTPUT_PATH, "training.csv")
noise_df = pd.DataFrame(data=train_noise_df, columns=NOISE_HEADING)
noise_df.to_csv(path_or_buf=out_file, index=False)

valid_df = valid_df.sample(frac=1)
out_file = os.path.join(OUTPUT_PATH, "validation.csv")
df = pd.DataFrame(data=valid_df, columns=HEADING)
df.to_csv(path_or_buf=out_file, index=False)

valid_noise_df = valid_noise_df.sample(frac=1)
out_file = os.path.join(NOISE_OUTPUT_PATH, "validation.csv")
noise_df = pd.DataFrame(data=valid_noise_df, columns=NOISE_HEADING)
noise_df.to_csv(path_or_buf=out_file, index=False)

test_df = test_df.sample(frac=1)
out_file = os.path.join(OUTPUT_PATH, "testing.csv")
df = pd.DataFrame(data=test_df, columns=HEADING)
df.to_csv(path_or_buf=out_file, index=False)

test_noise_df = test_noise_df.sample(frac=1)
out_file = os.path.join(NOISE_OUTPUT_PATH, "testing.csv")
noise_df = pd.DataFrame(data=test_noise_df, columns=NOISE_HEADING)
noise_df.to_csv(path_or_buf=out_file, index=False)