import os
import pandas
from tqdm import tqdm
from FELICE_cmd_map import FULL, OLD_CMD_LABEL_3, OLD_CMD_LABEL_3_AND_7, OLD_CMD_LABEL_7, OLD_CMD_LABEL_7_PLUS, PHASE_I_ROBOT


LANGS = ["eng", "ita"]
DEMO = "phase_I"  # ["full", "3", "7", "7_plus", "phase_I"]
IN_DATASET_FOLDER = "MIVIA_CRF_ISC"
#OUT_DATASET_FOLDER = os.path.join("FELICE", "demo"+DEMO)
OUT_DATASET_FOLDER = os.path.join("FELICE", "MIVIA_CRF_ISC_phaseI")
ANNOTATION_FILE = "dataset.csv"
COLUMNS = ["path", "type", "subtype", "speaker", "command"]
BALANCE_REJECT_SAMPLES = False
REJECT_PERCENTAGE = 0.75
'''
GOOGLE_USEFUL_COMMANDS = {os.path.join("dataset", "google_speech_commands_v1", "go"):   22,
                          os.path.join("dataset", "google_speech_commands_v1", "stop"): 24}
'''

if DEMO == str(3):
    OLD_CMD_LABEL = OLD_CMD_LABEL_3
elif DEMO == str(7):
    OLD_CMD_LABEL = OLD_CMD_LABEL_7
elif DEMO == "7_plus":
    OLD_CMD_LABEL = OLD_CMD_LABEL_7_PLUS
elif DEMO == "full":
    OLD_CMD_LABEL = FULL
elif DEMO == "phase_I":
    OLD_CMD_LABEL = PHASE_I_ROBOT

for lang in LANGS:
    in_dataset_folder = os.path.join("datasets", IN_DATASET_FOLDER, "annotations", lang)
    os.makedirs(in_dataset_folder, exist_ok=True)
    in_dataset_path = os.path.join(in_dataset_folder, ANNOTATION_FILE)
    out_dataset_folder = os.path.join("datasets", OUT_DATASET_FOLDER, "annotations", lang)
    os.makedirs(out_dataset_folder, exist_ok=True)
    out_dataset_path = os.path.join(out_dataset_folder, ANNOTATION_FILE)
    in_df = pandas.read_csv(filepath_or_buffer=in_dataset_path, sep=',')
    out_dict = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}

    df_iter = tqdm(in_df.index)
    for idx in df_iter:
        if in_df.iloc[idx]["command"] in OLD_CMD_LABEL:
            out_dict["path"].append(in_df.iloc[idx]["path"])
            out_dict["type"].append(in_df.iloc[idx]["type"])
            out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
            out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
            out_dict["command"].append(OLD_CMD_LABEL[in_df.iloc[idx]["command"]])
        elif DEMO == "full":
            if os.path.join(lang, "stop") in in_df.iloc[idx]["path"]:
                out_dict["path"].append(in_df.iloc[idx]["path"])
                out_dict["type"].append("google_speech_commands")
                out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
                out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
                out_dict["command"].append(24)
            elif lang == "eng":
                if os.path.join(lang, "up") in in_df.iloc[idx]["path"]:
                    out_dict["path"].append(in_df.iloc[idx]["path"])
                    out_dict["type"].append("google_speech_commands")
                    out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
                    out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
                    out_dict["command"].append(27)
                elif os.path.join(lang, "down") in in_df.iloc[idx]["path"]:
                    out_dict["path"].append(in_df.iloc[idx]["path"])
                    out_dict["type"].append("google_speech_commands")
                    out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
                    out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
                    out_dict["command"].append(28)
                elif os.path.join(lang, "left") in in_df.iloc[idx]["path"]:
                    out_dict["path"].append(in_df.iloc[idx]["path"])
                    out_dict["type"].append("google_speech_commands")
                    out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
                    out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
                    out_dict["command"].append(26)
                elif os.path.join(lang, "right") in in_df.iloc[idx]["path"]:
                    out_dict["path"].append(in_df.iloc[idx]["path"])
                    out_dict["type"].append("google_speech_commands")
                    out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
                    out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
                    out_dict["command"].append(25)
                else:
                    out_dict["path"].append(in_df.iloc[idx]["path"])
                    out_dict["type"].append("reject")
                    out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
                    out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
                    out_dict["command"].append(len(OLD_CMD_LABEL))
            else:
                out_dict["path"].append(in_df.iloc[idx]["path"])
                out_dict["type"].append("reject")
                out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
                out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
                out_dict["command"].append(len(OLD_CMD_LABEL))
        else:
            out_dict["path"].append(in_df.iloc[idx]["path"])
            out_dict["type"].append("reject")
            out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
            out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
            out_dict["command"].append(len(OLD_CMD_LABEL))

        df_iter.set_description("Building <{}> annotation file.".format(lang))

    '''
    # Add "go" and "stop" samples from Google Speech Command dataset
    if lang == "eng":
        for cmd_path in GOOGLE_USEFUL_COMMANDS:
            for path in os.listdir(cmd_path):
                out_dict["path"].append(in_df.iloc[idx]["path"])
                out_dict["type"].append("google_speech_commands")
                out_dict["subtype"].append(in_df.iloc[idx]["subtype"])
                out_dict["speaker"].append(in_df.iloc[idx]["speaker"])
                out_dict["command"].append(GOOGLE_USEFUL_COMMANDS[cmd_path])
    '''

    out_df = pandas.DataFrame(data=out_dict, columns=COLUMNS)

    if BALANCE_REJECT_SAMPLES:
        df_group = out_df.groupby(out_df.type)
        reject_group = df_group.get_group("reject")
        samples = len(out_df)
        reject_samples = len(reject_group)
        command_samples = samples - reject_samples
        reject_samples_to_insert = command_samples * REJECT_PERCENTAGE / (1 - REJECT_PERCENTAGE)
        reject_samples_to_remove = reject_samples - reject_samples_to_insert
        #print(samples, command_samples, reject_samples, reject_samples_to_insert, reject_samples_to_remove)
        out_df.sample(frac=1, ignore_index=True)
        df_iter = tqdm(out_df.index)
        for index in df_iter:
            if reject_samples_to_remove > 0 and out_df["type"][index] == "reject":
                out_df.drop(index=index, inplace=True)
                reject_samples_to_remove -= 1
            df_iter.set_description("Remove reject samples from <{}> dataset to obtain the 'reject/command samples rate' of {}.".format(lang, REJECT_PERCENTAGE))

    out_df.to_csv(path_or_buf=out_dataset_path, index=False)
    
in_noise_path = os.path.join("datasets", IN_DATASET_FOLDER, "annotations", "noise")
out_noise_path = os.path.join("datasets", OUT_DATASET_FOLDER, "annotations")
cmd = "cp -r {} {}".format(in_noise_path, out_noise_path)
os.system(command=cmd)