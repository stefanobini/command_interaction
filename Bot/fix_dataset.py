"""
Fix the data acquired through the Telegram both in qhich there are some commands with bad definition. This is caused by the wrong definition in the previous phase, so it is managed after acquisition through this script.
"""

import os
import sys
from tqdm import tqdm


DATASET_PATH = './saves'
ITA_CMDS_MV = [26, 27]
ENG_CMDS_RM = [26, 27]

renamed_sample = 0
deleted_sample = 0

user_iterator = tqdm(os.listdir(DATASET_PATH))
for user in user_iterator:
    user_path = os.path.join(DATASET_PATH, user)
    if os.path.isdir(user_path):
        for lang in os.listdir(user_path):
            lang_path = os.path.join(user_path, lang)
            if lang == 'ita':
                for command in ITA_CMDS_MV:
                    curr_name = 'ita_'+str(command)+'.ogg'
                    curr_path = os.path.join(lang_path, curr_name)
                    if 'ita_'+str(command)+'.ogg' in os.listdir(lang_path):
                        new_name = 'ita_'+str(command+5)+'.ogg'
                        new_path = os.path.join(lang_path, new_name)
                        if os.path.isfile(new_path):
                            os.remove(new_path)
                        os.rename(curr_path, new_path)
                        renamed_sample += 1
            elif lang == 'eng':
                for command in ENG_CMDS_RM:
                    name = 'eng_'+str(command)+'.ogg'
                    new_path = os.path.join(lang_path, name)
                    if os.path.isfile(new_path):
                        os.remove(new_path)
                        deleted_sample += 1
    user_iterator.set_description('Analizing')

print('Renamed samples: {}\nDeleted samples: {}'.format(renamed_sample, deleted_sample))