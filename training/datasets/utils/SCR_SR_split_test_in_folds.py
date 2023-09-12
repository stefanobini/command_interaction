"""Divide the test set into different folds in order to test the statistical validity of the results.
The folds will be disjoint and balanced with respect to the commands and SNR. Also, the application of noise to different
SNR is done after splitting so as not to have the same starting audio sample on different folds."""

import os
import pandas as pd
from tqdm import tqdm
from typing import List


DATASET_PATH = os.path. join("datasets", "MTL_experimentation_1")
ANNOTATION_FILE_NAME = "testing.csv"
ANNOTATION_PATH = os.path.join(DATASET_PATH, "annotations")
LANGs = ["eng", "ita"]
HEADINGS = ["path", "type", "subtype", "speaker", "command", "snr"]
N_FOLDS = 20


def split_dataframe(df:pd.DataFrame, n_folds:int) -> List[pd.DataFrame]:
    """Splid the test set in <n_fold> fold.

    Parameters
    ----------
    df: pandas.DataFrame
        Pandas dataframe containing the data to split
    n_fold: int
        Number of fold to create

    Returns
    -------
    List[pandas.DataFrame]
        List of <n_folds> pandas dataframe containing the test samples
    """
    n_sample = len(df)

    fold_dfs = list()
    for i in range(n_folds):
        fold_df = pd.DataFrame(columns=HEADINGS)
        fold_dfs.append(fold_df)

    start, end = 0, 0
    step =  (n_sample + n_folds // 2) // n_folds
    for i in range(n_folds):
        start = i * step
        end = (i+1) * step
        fold_df = df.iloc[start:end]
        #print(i, start, end, step, len(fold_df), n_sample)
        fold_dfs[i] = pd.concat(objs=[fold_dfs[i], fold_df])
    
    start = end
    fold_df = df.iloc[start:]
    fold_dfs[-1] = pd.concat(objs=[fold_dfs[-1], fold_df])

    return fold_dfs


for lang in LANGs:
    annotation_path = os.path.join(ANNOTATION_PATH, lang)
    annotation_file = os.path.join(annotation_path, ANNOTATION_FILE_NAME)

    ''' Read annotation file '''
    test_df = pd.read_csv(annotation_file, sep=',')

    ''' Shuffle the row of the dataframe '''
    test_df = test_df.sample(frac=1)
    # n_noisyspeech_samples = len(test_df)
    # n_snrs = len(test_df.groupby(test_df.snr).groups)
    # n_speech_samples = n_noisyspeech_samples // n_snrs

    ''' Create list of dataframe fold '''
    fold_dfs = list()
    for i in range(N_FOLDS):
        fold_df = pd.DataFrame(columns=HEADINGS)
        fold_dfs.append(fold_df)

    command_group = test_df.groupby(test_df.command)
    command_iter = tqdm(command_group.groups)

    # BALANCING ON THE COMMAND AMONG THE SETS
    for command in command_iter:
        command_df = command_group.get_group(command)
        command_df_sets = split_dataframe(df=command_df, n_folds=N_FOLDS)
        
        for fold in range(N_FOLDS):
            fold_dfs[fold] = pd.concat(objs=[fold_dfs[fold], command_df_sets[fold]])
                    
        command_iter.set_description("Splitting {} test set in {} folds".format(lang.upper(), N_FOLDS))

    for i in range(N_FOLDS):
        out_file = annotation_file.replace(".csv", "_fold{0:02d}.csv".format(i))
        fold_df = fold_dfs[i]#.sample(frac=1)
        # fold_df = pd.DataFrame(data=fold_df, columns=test_df.head())
        fold_df.to_csv(path_or_buf=out_file, index=False)