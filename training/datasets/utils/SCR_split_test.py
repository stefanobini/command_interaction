import os
import pandas


DATASET_NAME = "SCR_experimentation"
ANNOTATION_PATH = os.path.join("datasets", DATASET_NAME, "testing", "annotations")
HEADINGS = ["path", "type", "subtype", "speaker", "command", "noise_path", "noise_type", "noise_subtype", "snr"]
LANGs = ["eng", "ita"]
N_FOLDs = 8


for lang in LANGs:
    annotation_file = os.path.join(ANNOTATION_PATH, lang, "testing.csv")
    df = pandas.read_csv(annotation_file, sep=',').sample(frac=1)
    speaker_groups = df.groupby(df.speaker)
    speakers = list(speaker_groups.groups.keys())
    
    '''Create list of dataframe fold'''
    folds = list()
    for i in range(N_FOLDs):
        fold_df = pandas.DataFrame(columns=HEADINGS)
        folds.append(fold_df)
    
    '''Fill the folds'''
    if lang == "eng":
        for idx in range(len(speakers)):
            fold = idx%N_FOLDs
            folds[fold] = pandas.concat([folds[fold], df[df["speaker"] == speakers[idx]]])
    elif lang == "ita":
        for fold in range(N_FOLDs):
            folds[fold] = pandas.concat([folds[fold], df[df["speaker"] == speakers[fold*3]]])
            folds[fold] = pandas.concat([folds[fold], df[df["speaker"] == speakers[fold*3+1]]])
            folds[fold] = pandas.concat([folds[fold], df[df["speaker"] == speakers[fold*3+2]]])
            folds[fold] = pandas.concat([folds[fold], df[df["speaker"] == speakers[-(fold+1)]]])

    '''Save fold annotations'''
    for i in range(len(folds)):
        out_file = annotation_file.replace(".csv", "_fold{0:02d}.csv".format(i))
        fold_df = folds[i]
        fold_df.to_csv(path_or_buf=out_file, index=False)
