import pandas


def check_missing_id(speaker_list, set):
    sequential = len(speaker_list) == max(speaker_list)+1
    print("Are the ID of the {} set sequential? {}".format(set, sequential))
    if not sequential:
        for i in range(max(speaker_list)):
            if i not in speaker_list:
                print("{} is not in the {} set".format(i, set))


ANNOTATION_FILE_1 = "./datasets/MTL_experimentation_1/annotations/eng/training.csv"
ANNOTATION_FILE_2 = "./datasets/MTL_experimentation_1/annotations/eng/validation.csv"
ANNOTATION_FILE_3 = "./datasets/MTL_experimentation_1/annotations/eng/testing.csv"


df_1 = pandas.read_csv(filepath_or_buffer=ANNOTATION_FILE_1, sep=',')
df_2 = pandas.read_csv(filepath_or_buffer=ANNOTATION_FILE_2, sep=',')
df_3 = pandas.read_csv(filepath_or_buffer=ANNOTATION_FILE_3, sep=',')

speakers_1 = sorted(list(df_1["speaker"].unique()))
speakers_2 = sorted(list(df_2["speaker"].unique()))
speakers_3 = sorted(list(df_3["speaker"].unique()))
for speaker in speakers_1:
    if speaker not in speakers_2:
        print("<{}> is only in the second csv".format(speaker))
    if speaker not in speakers_3:
        print("<{}> is only in the third csv".format(speaker))
for speaker in speakers_2:
    if speaker not in speakers_1:
        print("<{}> is only in the first csv".format(speaker))
    if speaker not in speakers_3:
        print("<{}> is only in the third csv".format(speaker))
for speaker in speakers_3:
    if speaker not in speakers_1:
        print("<{}> is only in the first csv".format(speaker))
    if speaker not in speakers_2:
        print("<{}> is only in the second csv".format(speaker))

equal = speakers_1 == speakers_2 == speakers_3
print("Do the csv files contain the same speakers? ", equal)
check_missing_id(speaker_list=speakers_1, set="TRAINING")
check_missing_id(speaker_list=speakers_2, set="VALIDATION")
check_missing_id(speaker_list=speakers_3, set="TESTING")