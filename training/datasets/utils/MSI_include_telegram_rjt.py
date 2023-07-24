import os
import pandas
from intents import INTENTS, EXPLICIT_INTENTS, IMPLICIT_INTENTS, INTENTS_DICT_MSIEXP0, INTENTS_DICT_MSIEXP1


EXPERIMENTATION = "MSIexp1"
INPUT_DATASET = os.path.join("datasets", EXPERIMENTATION, "rejects")
OUTPUT_ANNOTATION_FILE = os.path.join("datasets", EXPERIMENTATION, "telegram_annotations_LANG.csv")
N_REJECTS = 10000
MIN_N_SAMPLExSPEAKER = 10
LANGs = ["eng", "esp", "ita"]
HEADING = ["path", "type", "subtype", "speaker", "intent", "explicit", "implicit"]

'''Set the heading for annotation file'''
if "MSIexp0" in EXPERIMENTATION:
    HEADING = ["path", "type", "subtype", "speaker", "command"]
elif "MSIexp1" in EXPERIMENTATION:
    HEADING = ["path", "type", "subtype", "speaker", "intent", "explicit", "implicit"]


for lang in LANGs:
    '''Set the reject label'''
    rjt_intents = len(INTENTS)-1
    rjt_exp_intents = INTENTS[rjt_intents]["explicit"][lang][0]["id"]
    rjt_imp_intents = 0
    data = dict()
    if "MSIexp0" in EXPERIMENTATION:
        rjt_intents = len(INTENTS_DICT_MSIEXP0)-1
        rjt_exp_intents = INTENTS_DICT_MSIEXP0[rjt_intents]["explicit"][lang][0]["id"]
        data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}
    elif "MSIexp1" in EXPERIMENTATION:
        rjt_intents = len(INTENTS_DICT_MSIEXP1)-1
        rjt_exp_intents = INTENTS_DICT_MSIEXP1[rjt_intents]["explicit"][lang][0]["id"]
        data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "intent": list(), "explicit": list(), "implicit": list()}

    lang_path = os.path.join(INPUT_DATASET, lang)
    for sample in os.listdir(lang_path):
        data["path"].append(os.path.join("rejects", lang, sample))
        data["type"].append("reject")
        data["subtype"].append("telegram")
        data["speaker"].append(sample.split('_')[1])
        
        '''Fill annotation file'''
        if "MSIexp0" in EXPERIMENTATION:
            data["command"].append(rjt_intents)
        elif "MSIexp1" in EXPERIMENTATION:
            data["intent"].append(rjt_intents)
            data["explicit"].append(rjt_exp_intents)
            data["implicit"].append(rjt_imp_intents)
    
    df = pandas.DataFrame(data=data, columns=HEADING)
    df.to_csv(path_or_buf=OUTPUT_ANNOTATION_FILE.replace("LANG", lang.upper()), index=False)
"""
sort_speaker_df = df.groupby(["speaker"])["path"].count().reset_index(name="count").sort_values(["count"], ascending=False).reset_index()

speakers = list()
accumulator = 0
for idx in sort_speaker_df.index:
    speaker = sort_speaker_df["speaker"][idx]
    n_speaker = sort_speaker_df["count"][idx]
    speakers.append(speaker)
    accumulator += n_speaker
    if accumulator > N_REJECTS or n_speaker < MIN_N_SAMPLExSPEAKER:
        break

out_df = df.loc[df["speaker"].isin(speakers)]

out_df.to_csv(path_or_buf=OUTPUT_ANNOTATION_FILE, index=False)
"""