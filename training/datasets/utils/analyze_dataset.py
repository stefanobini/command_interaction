"""
python3 datasets/utils/analyze_dataset.py
"""

import os
import re
import json
from tqdm import tqdm


LANGS = ["eng", "ita"]
DATASET_NAME = "MIVIA_ISC_v2"
DATASET_PATH = os.path.join("datasets", DATASET_NAME)

####################
###   COMMANDS   ###
####################

cmds = {
    'total': {'eng': 0, 'ita': 0},
    'class': {}
}
speakers = {
    'real': {'eng': 0, 'ita': 0},
    'synthetic': {'eng': 0, 'ita': 0}
}

cmd_path = os.path.join(DATASET_PATH, "commands")
user_iterator = tqdm(os.listdir(cmd_path))
for user in user_iterator:
    user_path = os.path.join(cmd_path, user)
    if os.path.isdir(user_path):
        for lang in os.listdir(user_path):
            lang_path = os.path.join(user_path, lang)
            if os.path.isdir(lang_path):
                speakers["real"][lang] += 1
                for file in os.listdir(lang_path):
                    if '.ogg' in file or ".wav" in file:
                        cmd = int(file.split('_')[-1].split('.')[0])
                        if cmds['class'] == {} or cmd not in cmds['class'].keys():
                            cmds['class'][cmd] = {'ita': 0, 'eng': 0}
                        cmds['total'][lang] += 1
                        cmds['class'][cmd][lang] += 1
    user_iterator.set_description('Analizing <command> samples')

out_path = os.path.join(DATASET_PATH, "cmd_analysis.json")
with open(out_path, 'w') as outfile:
    json.dump(sorted(cmds.items()), outfile)

out_path = os.path.join(DATASET_PATH, "spk_analysis.json")
with open(out_path, 'w') as outfile:
    json.dump(sorted(speakers.items()), outfile)


######################
###   SYNTHETICS   ###
######################

cmds = {
    'total': {'eng': 0, 'ita': 0},
    'class': {}
}
rex = "[0-9]*\.wav$"

syn_path = os.path.join(DATASET_PATH, "synthetics")
user_iterator = tqdm(os.listdir(syn_path))
for tts in user_iterator:
    tts_path = os.path.join(syn_path, tts)
    if os.path.isdir(tts_path):
        for mod in os.listdir(tts_path):
            mod_path = os.path.join(tts_path, mod)
            if os.path.isdir(mod_path):
                for lang in os.listdir(mod_path):
                    lang_path = os.path.join(mod_path, lang)
                    if os.path.isdir(lang_path):
                        for file in os.listdir(lang_path):
                            if '.wav' in file:
                                cmd_rex = re.search(pattern=rex, string=file)
                                cmd = cmd_rex.group().split(".")[0]
                                if cmds['class'] == {} or cmd not in cmds['class'].keys():
                                    cmds['class'][cmd] = {'ita': 0, 'eng': 0}
                                cmds['total'][lang] += 1
                                cmds['class'][cmd][lang] += 1
    user_iterator.set_description('Analizing <synthetic> samples')

out_path = os.path.join(DATASET_PATH, "syn_analysis.json")
with open(out_path, 'w') as outfile:
    json.dump(sorted(cmds.items()), outfile)


###################
###   REJECTS   ###
###################

rjts = {
    'total': {'eng': 0, 'ita': 0},
    'type': {
        "commands": {"ita": 0, "eng": 0},
        "synthetics": {"ita": 0, "eng": 0},
        "google": {"ita": 0, "eng": 0},
        "common_voices": {"ita": 0, "eng": 0},
        "other_noises": {"ita": 0, "eng": 0},
        "crf_noises": {"ita": 0, "eng": 0}
    }
}

rjt_path = os.path.join(DATASET_PATH, "rejects")
lang_iterator = tqdm(LANGS)
for lang in lang_iterator:
    lang_path = os.path.join(rjt_path, lang)
    if os.path.isdir(lang_path):
        for file in os.listdir(lang_path):
            if '.wav' in file:
                if "common_voice" in file:
                    type = "common_voices"
                elif "nohash" in file:
                    type = "google"
                elif lang in file:
                    type = "commands"
                elif "_background_noise_" in file:
                    type = "other_noises"
                else:
                    type = "synthetics"

                rjts['total'][lang] += 1
                rjts['type'][type][lang] += 1
    
    lang_iterator.set_description('Analizing <reject> samples')

noise_path = os.path.join(DATASET_PATH, "noises")
noise_iterator = tqdm(os.listdir(noise_path))
for folder in noise_iterator:
    folder_path = os.path.join(noise_path, folder)
    for subfolder in os.listdir(folder_path):
        subfolder_path = os.path.join(folder_path, subfolder)
        if os.path.isdir(subfolder_path):
            for subfile in os.listdir(subfolder_path):
                for lang in LANGS:
                    rjts['total'][lang] += 1
                    if folder == "crf_melfi":
                        rjts['type']["crf_noises"][lang] += 1
                    else:
                        rjts['type']["other_noises"][lang] += 1
        else:
            for lang in LANGS:
                rjts['total'][lang] += 1
                rjts['type']["other_noises"][lang] += 1
    noise_iterator.set_description('Analizing <noise> samples')

out_path = os.path.join(DATASET_PATH, "rjt_analysis.json")
with open(out_path, 'w') as outfile:
    json.dump(sorted(rjts.items()), outfile)