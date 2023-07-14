import os
import json
from tqdm import tqdm


DATASET_PATH = 'recordings'
OUTPUT_PATH = 'intents.json'

cmds = {
    "speakers": {"eng":0, "esp":0, "ita":0},
    "total": {"eng":0, "esp":0, "ita":0},
    "class": {}
}

user_iterator = tqdm(os.listdir(DATASET_PATH))
for user in user_iterator:
    user_path = os.path.join(DATASET_PATH, user)
    if os.path.isdir(user_path):
        for lang in os.listdir(user_path):
            lang_path = os.path.join(user_path, lang)
            if os.path.isdir(lang_path):
                cmds["speakers"][lang] += 1
                for file in os.listdir(lang_path):
                    if '.ogg' in file:
                        intents = int(file.split('_')[0])
                        if cmds['class'] == {} or intents not in cmds['class'].keys():
                            cmds['class'][intents] = {'eng': 0, "esp": 0, 'ita': 0}
                        cmds['total'][lang] += 1
                        cmds['class'][intents][lang] += 1
    user_iterator.set_description('Analizing')

print('Approximate number of speakers:')
for lang in cmds['total']:
    print("{}: {}".format(lang.upper(), int(cmds['total'][lang]/len(cmds['class'])/3)))

with open(OUTPUT_PATH, 'w') as outfile:
    json.dump(sorted(cmds.items()), outfile)