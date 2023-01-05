import os
import random
import pandas as pd
from tqdm import tqdm


HEADING = ["path", "type", "subtype", "speaker", "label"]
LANG = "ita"

DATASET_PATH = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1"
ANNOTATIONS_FILE = os.path.join(DATASET_PATH, "annotations/{}/{}_dataset.csv".format(LANG, LANG))
OUTPUT_PATH = os.path.join(DATASET_PATH, "annotations/{}/".format(LANG))

TRAINING = 0.8
VALIDATION = 0.1
TEST = 0.1


def split_dataframe(df:pd.DataFrame, train_rate:float, valid_rate:float, test_rate:float) -> tuple:
    """Splid a dataframe in training, validation and test set.

    Parameters
    ----------
    df: pandas.DataFrame
        Pandas dataframe containing the data to split
    train_rate: float
        Relatove percentage of the sample to include in training set
    valid_rate: float
        Relatove percentage of the sample to include in validation set
    test_rate: float
        Relatove percentage of the sample to include in test set

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

    train_df = df.iloc[:int(n_sample*train_rate)]
    validation_df = df.iloc[int(n_sample*train_rate):int(n_sample*train_rate)+int(n_sample*valid_rate)]
    test_df = df.iloc[int(n_sample*train_rate)+int(n_sample*valid_rate):]

    return train_df, validation_df, test_df


''' Read annotation file '''
df = pd.read_csv(ANNOTATIONS_FILE, sep=',')

''' Shuffle the row of the dataframe '''
df = df.sample(frac=1)

''' Split dataset in training, validation and test set '''
n_sample = len(df)

type_group = df.groupby(df.type)
for type in type_group.groups:
    df_type = type_group.get_group(type)
    df_type = df_type.sample(frac=1)
    df_type_train, df_type_valid, df_type_test = split_dataframe(df=df_type, train_rate=TRAINING, valid_rate=VALIDATION, test_rate=TEST)
    speaker_group = df_type.groupby()

train_df = df.iloc[:int(n_sample*TRAINING)]
validation_df = df.iloc[int(n_sample*TRAINING):int(n_sample*TRAINING)+int(n_sample*VALIDATION)]
test_df = df.iloc[int(n_sample*TRAINING)+int(n_sample*VALIDATION):]

''' WRITE CSV FILES '''
out_file = os.path.join(OUTPUT_PATH, "{}_train.csv".format(LANG))
df = pd.DataFrame(data=train_df, columns=HEADING)
df.to_csv(path_or_buf=out_file, index=False)

out_file = os.path.join(OUTPUT_PATH, "{}_valid.csv".format(LANG))
df = pd.DataFrame(data=validation_df, columns=HEADING)
df.to_csv(path_or_buf=out_file, index=False)

out_file = os.path.join(OUTPUT_PATH, "{}_test.csv".format(LANG))
df = pd.DataFrame(data=test_df, columns=HEADING)
df.to_csv(path_or_buf=out_file, index=False)