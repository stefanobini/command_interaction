import os
import shutil
import pandas as pd
from tqdm import tqdm
from math import floor
from typing import Tuple


LANGS = ["eng", "ita"]

INPUT_DATASET_PATH = os.path.join("datasets", "MSI")
INPUT_NOISE_PATH = os.path.join("datasets", "MSI", "annotations", "noise")
OUTPUT_DATASET_PATH = os.path.join("datasets", "MSI")
os.makedirs(OUTPUT_DATASET_PATH, exist_ok=True)
OUTPUT_NOISE_PATH = os.path.join(OUTPUT_DATASET_PATH, "annotations", "noise")
os.makedirs(OUTPUT_NOISE_PATH, exist_ok=True)

HEADING = ["path", "type", "subtype", "speaker", "intent", "explicit", "implicit"]
NOISE_HEADING = ["path", "type", "subtype"]

TRAINING = 0.8
VALIDATION = 0.1
TEST = 0.1


def split_dataframe(df:pd.DataFrame, train_rate:float, valid_rate:float, test_rate:float) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """Splid a dataframe in training, validation and test set. The splitting is balanced on the speaker.

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
    annotation_file = os.path.join(INPUT_DATASET_PATH, "annotations", lang, "dataset.csv")
    output_path = os.path.join(OUTPUT_DATASET_PATH, "annotations", lang)
    os.makedirs(output_path, exist_ok=True)

    ''' Read annotation file '''
    df = pd.read_csv(annotation_file, sep=',')

    ''' Shuffle the row of the dataframe '''
    df = df.sample(frac=1)

    ''' Split dataset in training, validation and test set '''
    n_sample = len(df)
    train_df = pd.DataFrame(columns=HEADING)
    valid_df = pd.DataFrame(columns=HEADING)
    test_df = pd.DataFrame(columns=HEADING)

    # BALANCING ON THE TYPE AMONG THE SETS
    type_group = df.groupby(df.type)
    type_iter = tqdm(type_group.groups)
    for type in type_iter:
        df_type = type_group.get_group(type)
        df_type = df_type.sample(frac=1)
        # Balance on the type among the set
        df_type_sets = split_dataframe(df=df_type, train_rate=TRAINING, valid_rate=VALIDATION, test_rate=TEST)

        for df_type_set in df_type_sets:
            subtype_group = df_type_set.groupby(df.subtype)

            # BALANCING ON THE SUPTYPE AMONG THE SETS
            for subtype in subtype_group.groups:
                df_subtype = subtype_group.get_group(subtype)
                df_subtype = df_subtype.sample(frac=1)
                df_subtype_sets = split_dataframe(df=df_subtype, train_rate=TRAINING, valid_rate=VALIDATION, test_rate=TEST)

                for df_subtype_set in df_subtype_sets:
                    speaker_group = df_subtype_set.groupby(df.speaker)
                    
                    # BALANCING ON THE SPEAKER AMONG THE SETS
                    for speaker in speaker_group.groups:
                        df_speaker = speaker_group.get_group(speaker)
                        df_speaker = df_speaker.sample(frac=1)
                        df_speaker_sets = split_dataframe(df=df_speaker, train_rate=TRAINING, valid_rate=VALIDATION, test_rate=TEST)

                        train_df = pd.concat(objs=[train_df, df_speaker_sets[0]])
                        valid_df = pd.concat(objs=[valid_df, df_speaker_sets[1]])
                        test_df = pd.concat(objs=[test_df, df_speaker_sets[2]])
                        
        type_iter.set_description("Analyzing <{}> samples for <{}> dataset.".format(type, lang))


    ''' WRITE CSV FILES '''
    train_df = train_df.sample(frac=1)
    out_file = os.path.join(output_path, "training.csv")
    train_df.to_csv(path_or_buf=out_file, index=False)

    valid_df["snr"] = [40 for i in range(len(valid_df))]
    valid_df = valid_df.sample(frac=1)
    out_file = os.path.join(output_path, "validation.csv")
    valid_df.to_csv(path_or_buf=out_file, index=False)

    test_df["snr"] = [40 for i in range(len(test_df))]
    test_df = test_df.sample(frac=1)
    out_file = os.path.join(output_path, "testing.csv")
    test_df.to_csv(path_or_buf=out_file, index=False)


############
#   NOISE  #
############
''' Copy all NOISE annotation files '''
# fetch all files
"""
for file_name in os.listdir(INPUT_NOISE_PATH):
    # construct full file path
    source = os.path.join(INPUT_NOISE_PATH, file_name)
    destination = os.path.join(OUTPUT_NOISE_PATH, file_name)
    # copy only files
    if os.path.isfile(source):
        shutil.copy(source, destination)
"""