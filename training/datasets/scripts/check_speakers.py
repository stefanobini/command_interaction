import pandas


ANNOTATION_FILE_1 = "./datasets/MTL_scr_sid/annotations/ita/training.csv"
ANNOTATION_FILE_2 = "./datasets/MTL_scr_sid/annotations/ita/validation.csv"


df_1 = pandas.read_csv(filepath_or_buffer=ANNOTATION_FILE_1, sep=',')
df_2 = pandas.read_csv(filepath_or_buffer=ANNOTATION_FILE_2, sep=',')

speakers_1 = df_1["speaker"].unique()
speakers_2 = df_2["speaker"].unique()
for speaker in speakers_1:
    if speaker not in speakers_2:
        print("<{}> is only in the second csv".format(speaker))
for speaker in speakers_2:
    if speaker not in speakers_1:
        print("<{}> is only in the first csv".format(speaker))

equal = set(speakers_1) == set(speakers_2)
print("Do the csv contain the same speakers? ", equal)