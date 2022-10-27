import os
import re
import json
from tqdm import tqdm


CMD_IN_PATH = './commands'
CMD_OUT_PATH = './commands_analysis.json'
SYN_IN_PATH = './synthetics'
SYN_OUT_PATH = './synthetics_analysis.json'
REJ_IN_PATH = './rejects'
REJ_OUT_PATH = './rejects_analysis.json'


####################
###   COMMANDS   ###
####################

cmds = {
    'total': {'eng': 0, 'ita': 0},
    'class': {}
}

user_iterator = tqdm(os.listdir(CMD_IN_PATH))
for user in user_iterator:
    user_path = os.path.join(CMD_IN_PATH, user)
    if os.path.isdir(user_path):
        for lang in os.listdir(user_path):
            lang_path = os.path.join(user_path, lang)
            if os.path.isdir(lang_path):
                for file in os.listdir(lang_path):
                    if '.ogg' in file or ".wav" in file:
                        cmd = int(file.split('_')[1].split('.')[0])
                        if cmds['class'] == {} or cmd not in cmds['class'].keys():
                            cmds['class'][cmd] = {'ita': 0, 'eng': 0}
                        cmds['total'][lang] += 1
                        cmds['class'][cmd][lang] += 1
    user_iterator.set_description('Analizing')

with open(CMD_OUT_PATH, 'w') as outfile:
    json.dump(sorted(cmds.items()), outfile)


######################
###   SYNTHETICS   ###
######################

cmds = {
    'total': {'eng': 0, 'ita': 0},
    'class': {}
}
rex = "[0-9]*\.wav$"

user_iterator = tqdm(os.listdir(SYN_IN_PATH))
for tts in user_iterator:
    tts_path = os.path.join(SYN_IN_PATH, tts)
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
    user_iterator.set_description('Analizing')

with open(SYN_OUT_PATH, 'w') as outfile:
    json.dump(sorted(cmds.items()), outfile)


###################
###   REJECTS   ###
###################

cmds = {
    'total': {'eng': 0, 'ita': 0},
    'type': {
        "commands": {"ita": 0, "eng": 0},
        "synthetics": {"ita": 0, "eng": 0},
        "google": {"ita": 0, "eng": 0},
        "common_voices": {"ita": 0, "eng": 0},
        "backgrounds": {"ita": 0, "eng": 0}
    }
}

lang_iterator = tqdm(["eng", "ita"])
for lang in lang_iterator:
    lang_path = os.path.join(REJ_IN_PATH, lang)
    if os.path.isdir(lang_path):
        lang_path = os.path.join(lang_path, "clips")
        for file in os.listdir(lang_path):
            if '.wav' in file:
                if "common_voice" in file:
                    type = "common_voices"
                elif "nohash" in file:
                    type = "google"
                elif lang in file:
                    type = "commands"
                elif "_background_noise_" in file:
                    type = "backgrounds"
                else:
                    type = "synthetics"

                cmds['total'][lang] += 1
                cmds['type'][type][lang] += 1
    
    lang_iterator.set_description('Analizing')

with open(REJ_OUT_PATH, 'w') as outfile:
    json.dump(sorted(cmds.items()), outfile)