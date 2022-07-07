import os
import sys
import json
from tqdm import tqdm


DATASET_PATH = './saves'
OUTPUT_PATH = './commands.json'

cmds = {
    'total': {'eng': 0, 'ita': 0},
    'class': {}
}

user_iterator = tqdm(os.listdir(DATASET_PATH))
for user in user_iterator:
    user_path = os.path.join(DATASET_PATH, user)
    if os.path.isdir(user_path):
        for lang in os.listdir(user_path):
            lang_path = os.path.join(user_path, lang)
            if os.path.isdir(lang_path):
                for file in os.listdir(lang_path):
                    if '.ogg' in file:
                        cmd = int(file.replace(lang+'_', '').replace('.ogg', ''))
                        if cmds['class'] == {} or cmd not in cmds['class'].keys():
                            cmds['class'][cmd] = {'ita': 0, 'eng': 0}
                        cmds['total'][lang] += 1
                        cmds['class'][cmd][lang] += 1
    user_iterator.set_description('Analizing')

with open(OUTPUT_PATH, 'w') as outfile:
    json.dump(sorted(cmds.items()), outfile)