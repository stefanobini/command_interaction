"""Divide the test set into different folds in order to test the statistical validity of the results.
The folds will be disjoint and balanced with respect to the commands and SNR. Also, the application of noise to different
SNR is done after splitting so as not to have the same starting audio sample on different folds."""

import os
import pandas as pd
from tqdm import tqdm
import random


LANGUAGE = "ita"
DATASET_PATH = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/final_dataset"
ANNOTATION_FILE = "class_testing_0_40.csv"
ANNOTATION_FILE_PATH = os.path.join(DATASET_PATH, "testing", "annotations", LANGUAGE, ANNOTATION_FILE)
OUTPUT_PATH = os.path.join(DATASET_PATH, "testing", "annotations", LANGUAGE)
N_FOLDS = 20
HEADINGS = ["path", "type", "subtype", "speaker", "label", "noise_path", "noise_type", "noise_subtype", "snr"]


''' Read annotation file '''
test_df = pd.read_csv(ANNOTATION_FILE_PATH, sep=',')

''' Shuffle the row of the dataframe '''
# test_df = test_df.sample(frac=1)
n_noisyspeech_samples = len(test_df)
n_snrs = len(test_df.groupby(test_df.snr).groups)
# n_speech_samples = n_noisyspeech_samples // n_snrs

''' Create list of dataframe fold '''
folds = list()
for i in range(N_FOLDS):
    fold_df = pd.DataFrame(columns=HEADINGS)
    folds.append(fold_df)

''' Split test set '''
count = 0
label_group = test_df.groupby(test_df.label)
label_iter = tqdm(label_group.groups)
for label in label_iter:
    df_label = label_group.get_group(label)
    # df_label = df_label.sample(frac=1)  # shuffle set
    n_label_sample = len(df_label)

    start, end = 0, 0
    step =  (n_label_sample + N_FOLDS // 2) // N_FOLDS
    for i in range(len(folds)-1):
        start = i * step
        end = (i+1) * step
        fold_df = df_label.iloc[start:end]
        folds[i] = pd.concat(objs=[folds[i], fold_df])
    
    start = end
    fold_df = df_label.iloc[start:]
    folds[-1] = pd.concat(objs=[folds[-1], fold_df])

    label_iter.set_description("Splitting {} in {} folds".format(ANNOTATION_FILE, N_FOLDS))

for i in range(len(folds)):
    name = ANNOTATION_FILE.replace(".csv", "_fold{0:02d}.csv".format(i))
    fold_df = folds[i]#.sample(frac=1)
    out_file = os.path.join(OUTPUT_PATH, name)
    # fold_df = pd.DataFrame(data=fold_df, columns=test_df.head())
    fold_df.to_csv(path_or_buf=out_file, index=False)