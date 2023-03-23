import os
import random
import pandas as pd
from tqdm import tqdm
from math import floor
from typing import Tuple


LANGS = ["eng", "ita"]
DATASET_PATH = "./datasets/MTL_experimentation"
NOISE_OUTPUT_PATH = os.path.join(DATASET_PATH, "annotations", "noise")
if not os.path.isdir(NOISE_OUTPUT_PATH):
        os.makedirs(NOISE_OUTPUT_PATH)
NOISE_ANNOTATIONS_FILE = os.path.join(NOISE_OUTPUT_PATH, "dataset.csv")
HEADING = ["path", "type", "subtype", "speaker", "label"]
NOISE_HEADING = ["path", "type", "subtype"]

TRAINING = 0.7
TEST = 0.3


def split_dataframe(df:pd.DataFrame, train_rate:float, test_rate:float) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
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

    num_test = floor(n_sample*test_rate) if (floor(n_sample*test_rate)>0 or n_sample<3) else 1
    num_train = n_sample - num_test

    train_df = df.iloc[:num_train]
    test_df = df.iloc[num_train:]

    return train_df, test_df



def get_df_attribute_set(attribute_group, attribute_list:list, heading) -> pd.DataFrame:
    df_set = pd.DataFrame(columns=heading)

    for attribute in attribute_list:
        df_attribute = attribute_group.get_group(attribute)
        df_attribute = df_attribute.sample(frac=1)

        df_set = pd.concat(objs=[df_set, df_attribute])

    return df_set

for lang in LANGS:
    annotation_file = os.path.join(DATASET_PATH, "annotations/{}/dataset.csv".format(lang))
    output_file = os.path.join(DATASET_PATH, "annotations/{}/".format(lang))
    ''' Read annotation file '''
    df = pd.read_csv(annotation_file, sep=',')

    ''' Shuffle the row of the dataframe '''
    df = df.sample(frac=1)

    ''' Split dataset in training, validation and test set '''
    n_sample = len(df)
    train_df = pd.DataFrame(columns=HEADING)
    test_df = pd.DataFrame(columns=HEADING)

    count = 0
    type_group = df.groupby(df.type)

    # BALANCING ON THE TYPE AMONG THE SETS
    for type in type_group.groups:
        df_type = type_group.get_group(type)
        df_type = df_type.sample(frac=1)
        # Balance on the type among the set
        df_type_sets = split_dataframe(df=df_type, train_rate=TRAINING, test_rate=TEST)
        # print("The TYPE '{}' contains '{}' samples.\nTrain:\t{}\nValid:\t{}\nTest:\t{}".format(type, len(df_type), len(df_type_sets[0]), len(df_type_sets[1]), len(df_type_sets[2])))

        for df_type_set in df_type_sets:
            subtype_group = df_type_set.groupby(df.subtype)

            # BALANCING ON THE SUPTYPE AMONG THE SETS
            for subtype in subtype_group.groups:
                df_subtype = subtype_group.get_group(subtype)
                df_subtype = df_subtype.sample(frac=1)
                df_subtype_sets = split_dataframe(df=df_subtype, train_rate=TRAINING, test_rate=TEST)
                # print("The TYPE {} SUBTYPE '{}' contains '{}' samples.\nTrain:\t{}\nValid:\t{}\nTest:\t{}".format(type, subtype, len(df_subtype), len(df_subtype_sets[0]), len(df_subtype_sets[1]), len(df_subtype_sets[2])))

                for df_subtype_set in df_subtype_sets:
                    speaker_group = df_subtype_set.groupby(df.speaker)
                    
                    if type != "reject":
                        n_speaker = len(speaker_group.groups)
                        num_test_speaker = floor(n_speaker*TEST) if (floor(n_speaker*TEST)>0 or n_speaker<3) else 1
                        num_train_speaker = n_speaker - num_test_speaker

                        train_speakers = list(speaker_group.groups.keys())[:num_train_speaker]
                        test_speakers = list(speaker_group.groups.keys())[num_train_speaker:]

                        train_df = pd.concat(objs=[train_df, get_df_attribute_set(attribute_group=speaker_group, attribute_list=train_speakers, heading=HEADING)])
                        test_df = pd.concat(objs=[test_df, get_df_attribute_set(attribute_group=speaker_group, attribute_list=test_speakers, heading=HEADING)])
                    else:
                        # BALANCING ON THE SPEAKER AMONG THE SETS
                        for speaker in speaker_group.groups:
                            df_speaker = speaker_group.get_group(speaker)
                            df_speaker = df_speaker.sample(frac=1)
                            df_speaker_sets = split_dataframe(df=df_speaker, train_rate=TRAINING, test_rate=TEST)

                            train_df = pd.concat(objs=[train_df, df_speaker_sets[0]])
                            test_df = pd.concat(objs=[test_df, df_speaker_sets[1]])


    ''' WRITE CSV FILES '''
    train_df = train_df.sample(frac=1)
    out_file = os.path.join(output_file, "training.csv")
    df = pd.DataFrame(data=train_df, columns=HEADING)
    df.to_csv(path_or_buf=out_file, index=False)

    test_df = test_df.sample(frac=1)
    out_file = os.path.join(output_file, "testing.csv")
    df = pd.DataFrame(data=test_df, columns=HEADING)
    df.to_csv(path_or_buf=out_file, index=False)



''' Read NOISE annotation file '''
noise_df = pd.read_csv(NOISE_ANNOTATIONS_FILE, sep=',')

''' Shuffle the row of the NOISE dataframe '''
noise_df = noise_df.sample(frac=1)

''' Split NOISE dataset in training, validation and test set '''
n_sample = len(noise_df)
train_noise_df = pd.DataFrame(columns=NOISE_HEADING)
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
test_list = [
    ("crf_melfi", "human_talk"),
    ("strumenti di lavoro (lavorazione metalli)", "tornio"),
    ("crf_melfi", "move"),
    ("catena di lavoro robot", "unknown"),
    ("strumenti di lavoro (lavorazione metalli)", "macchina per incidere"),
    ("strumenti di lavoro (lavorazione metalli)", "raschiatura"),
    ("unknown", "unknown")
]

train_noise_df = pd.concat(objs=[train_noise_df, get_df_attribute_set(attribute_group=type_subtype_group, attribute_list=train_list, heading=NOISE_HEADING)])
test_noise_df = pd.concat(objs=[test_noise_df, get_df_attribute_set(attribute_group=type_subtype_group, attribute_list=test_list, heading=NOISE_HEADING)])

# print("Counting: {}\nTraining: {}\nValidation: {}\nTesting: {}\n".format(type_subtype_group.count(), len(train_noise_df), len(valid_noise_df), len(test_noise_df)))
#################

''' WRITE CSV FILES '''
train_noise_df = train_noise_df.sample(frac=1)
out_file = os.path.join(NOISE_OUTPUT_PATH, "training.csv")
noise_df = pd.DataFrame(data=train_noise_df, columns=NOISE_HEADING)
noise_df.to_csv(path_or_buf=out_file, index=False)

test_noise_df = test_noise_df.sample(frac=1)
out_file = os.path.join(NOISE_OUTPUT_PATH, "testing.csv")
noise_df = pd.DataFrame(data=test_noise_df, columns=NOISE_HEADING)
noise_df.to_csv(path_or_buf=out_file, index=False)