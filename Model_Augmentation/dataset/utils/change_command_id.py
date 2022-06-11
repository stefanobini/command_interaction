import os
from tqdm import tqdm
import re


DATASET_PATH = '../FELICE_demo7'
CMD_PATH = os.path.join(DATASET_PATH, 'commands')
SYN_PATH= os.path.join(DATASET_PATH, 'synthetics')
CMDS_DICT = {
    6:0,
    8:1,
    12:2,
    16:3,
    18:4,
    20:5,
    23:6
}# 'old_id':'new_id'
CMDS_LIST = CMDS_DICT.keys()

SETS = {
    'real': CMD_PATH,
    'azure': os.path.join(SYN_PATH, 'azure'),
    'nemo': os.path.join(SYN_PATH, 'nemo'),
    'polly': os.path.join(SYN_PATH, 'polly')
}


def extract_cmd_id(set, file_name):
    digits = re.findall('[0-9]+', file_name)
    if set in ['real', 'azure', 'nemo']:
        return int(digits[-1])
    elif set == 'polly':
        return int(digits[-2])


def change_name(set, file_name, cmd):
    if set in ['real', 'azure', 'nemo']:
        return os.rename(file_name, file_name.replace(str(cmd)+'.wav', str(CMDS_DICT[cmd])+'.wav'))
    elif set == 'polly':
        return os.rename(file_name, file_name.replace(str(cmd)+'.mp3', str(CMDS_DICT[cmd])+'.mp3'))


def change_cmd_id(set, path):
    user_iterator = tqdm(os.listdir(path))
    for user in user_iterator:
        user_path = os.path.join(path, user)
        if os.path.isdir(user_path):
            for lang in os.listdir(user_path):
                lang_path = os.path.join(user_path, lang)
                if os.path.isdir(lang_path):
                    for file in os.listdir(lang_path):
                        # print(lang_path)
                        cmd = extract_cmd_id(set, file)
                        sample_path = os.path.join(lang_path, file)
                        change_name(set, sample_path, cmd)
        user_iterator.set_description('Analizing {} samples'.format(set))


for set, path in SETS.items():
    change_cmd_id(set, path)

print('Command IDs changed!')