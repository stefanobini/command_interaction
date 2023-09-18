"""
python3 datasets/utils/add_additional_info.py 
"""

import os
import pandas
from tqdm import tqdm
from typing import List


DATASET_NAME = "MTL_experimentation_1"
DATASET_ANNOTATION_PATH = os.path.join("datasets", DATASET_NAME, "annotations")
ADDITIONAL_INFO_PATH = os.path.join("datasets", "MIVIA_ISC_v2", "additional_info.csv")
LANGS = ["eng", "ita"]
DATASET_HEADING = ["path", "type", "subtype", "speaker", "command"]
ADDITIONAL_HEADING = ["speaker", "gender", "age", "eng_cmds", "ita_cmds"]
FULL_HEADING = ["path", "type", "subtype", "speaker", "command", "gender", "age"]

"""SYNTHETIC SAMPLES"""
MALE_SPEAKERS = ["BrandonNeural", "ChristopherNeural", "ConnorNeural", "EricNeural", "JacobNeural", "JamesNeural", "LiamNeural", "LukeNeural", "MitchellNeural", "PrabhatNeural", "RyanNeural", "SamNeural", "WayneNeural", "WilliamNeural", "DiegoNeural", "charles", "grahm", "mike", "peter", "rod", "ryan", "tyler", "will", "vittorio", "Brian", "Joey", "Justin", "Kevin", "Matthew", "Brian", "Geraint", "Russel"]
FEMALE_SPEAKERS = ["AmberNeural", "AriaNeural", "AshleyNeural", "ClaraNeural", "CoraNeural", "ElizabethNeural", "EmilyNeural", "JennyNeural", "LeahNeural", "LibbyNeural", "LunaNeural", "MiaNeural", "MichelleNeural", "MollyNeural", "MonicaNeural", "NatashaNeural", "NeerjaNeural", "RosaNeural", "YanNeural", "ElsaNeural", "IsabellaNeural", "audrey", "heather", "karen", "laura", "lisa", "lucy", "rachel", "sharon", "tracy", "Amy", "Emma", "Ivy", "Joanna", "Kendra", "Kimberly", "Olivia", "Salli", "Aditi", "Nicole", "Raveena"]
VOCALWARE_SUBTYPE = "vocalware_neural"
VOCALWARE_MALE = ["eng/engine2_voice2", "eng/engine2_voice3", "eng/engine2_voice5", "eng/engine2_voice8", "eng/engine2_voice9", "eng/engine3_voice2", "eng/engine3_voice5", "eng/engine3_voice7", "eng/engine7_voice2", "eng/engine7_voice4", "eng/engine7_voice6", "ita/engine2_voice4", "ita/engine2_voice5", "ita/engine2_voice6", "ita/engine2_voice7", "ita/engine2_voice8", "ita/engine3_voice2", "ita/engine7_voice2"]
VOCALWARE_FEMALE = ["eng/engine2_voice1", "eng/engine2_voice4", "eng/engine2_voice6", "eng/engine2_voice7", "eng/engine2_voice10", "eng/engine2_voice11", "eng/engine3_voice1", "eng/engine3_voice3", "eng/engine3_voice6", "eng/engine3_voice8", "eng/engine7_voice1", "eng/engine7_voice3", "eng/engine7_voice5", "ita/engine2_voice1", "ita/engine2_voice2", "ita/engine2_voice3", "ita/engine2_voice9", "ita/engine2_voice10", "ita/engine3_voice1", "ita/engine7_voice1"]
NEMO_SUBTYPE = "nemo_neural" # female
GOOGLE_SUBTYPE = ["google_neural", "google_standard"]
GOOGLE_ENG = "Male"
GOOGLE_ITA = "Female"


additional_info_df = pandas.read_csv(ADDITIONAL_INFO_PATH, sep=',', dtype=str)
additional_info_df = additional_info_df.sample(frac=1)
full_dataset = {"path":list(), "type":list(), "subtype":list(), "speaker":list(), "command":list(), "gender":list(), "age":list()}

for lang in LANGS:
    #lang_dataset_annotation_path = os.path.join(DATASET_ANNOTATION_PATH, lang, "fixed_dataset.csv")
    lang_dataset_annotation_path = os.path.join(DATASET_ANNOTATION_PATH, lang, "dataset_no_speaker_id_0.csv")
    dataset_df = pandas.read_csv(lang_dataset_annotation_path, sep=',', dtype=str)
    dataset_df = dataset_df.sample(frac=1)
    sample_iter = tqdm(dataset_df.index)
    for i in sample_iter:
        gender = None
        age = None
        if dataset_df["type"][i] == "command":
            # Find the speaker information of the current sample of the original annotation dataset file
            speaker_info_df = additional_info_df.loc[additional_info_df["speaker"] == dataset_df["speaker"][i]]
            if len(speaker_info_df) > 0:
                gender = speaker_info_df.iloc[0]["gender"]
                age = speaker_info_df.iloc[0]["age"]
            else:
                gender = None
                age = None
        elif dataset_df["type"][i] == "synthetic":
            # Handle Nemo synthetic samples
            if dataset_df["subtype"][i] == NEMO_SUBTYPE:
                gender = "Female"
            # Handle Google synthetic samples
            elif dataset_df["subtype"][i] in GOOGLE_SUBTYPE:
                if "/eng/" in dataset_df["path"][i]:
                    gender = GOOGLE_ENG
                elif "/ita/" in dataset_df["path"][i]:
                    gender = GOOGLE_ITA
            # Handle VocalWare synthetic samples
            elif dataset_df["subtype"][i] in VOCALWARE_SUBTYPE:
                for e in VOCALWARE_MALE:
                    gender = "Male" if e in dataset_df["path"][i] else "Female"
            else:
                for e in MALE_SPEAKERS:
                    gender = "Male" if e in dataset_df["speaker"][i] else "Female"

        # Fill the new annotation file with all the speaker information
        full_dataset["path"].append(dataset_df["path"][i])
        full_dataset["type"].append(dataset_df["type"][i])
        full_dataset["subtype"].append(dataset_df["subtype"][i])
        full_dataset["speaker"].append(dataset_df["speaker"][i])
        full_dataset["command"].append(dataset_df["command"][i])
        full_dataset["gender"].append(gender)
        full_dataset["age"].append(age)
        sample_iter.set_description("Analyzing {} dataset".format(lang.upper()))
    
    full_dataset_df = pandas.DataFrame(data=full_dataset, columns=FULL_HEADING)
    full_dataset_df = full_dataset_df.sample(frac=1)
    #full_dataset_df.to_csv(path_or_buf=lang_dataset_annotation_path.replace("fixed_", ""), index=False)
    full_dataset_df.to_csv(path_or_buf=lang_dataset_annotation_path.replace("_0", ""), index=False)