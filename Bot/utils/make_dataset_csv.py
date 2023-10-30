import os
import sys
import csv
import json
from tqdm import tqdm


DATASET_PATH = './saves'
OUTPUT_PATH = './commands.csv'

users = list()

user_iterator = tqdm(os.listdir(DATASET_PATH))
for user_folder in user_iterator:
    user_path = os.path.join(DATASET_PATH, user_folder)
    if os.path.isdir(user_path):
        user = {'user': user_folder, 'ita': list(), 'eng': list()}
        for lang in os.listdir(user_path):
            lang_path = os.path.join(user_path, lang)
            if os.path.isdir(lang_path):
                cmd_list = list()
                for file in os.listdir(lang_path):
                    if '.ogg' in file:
                        cmd = int(file.replace(lang+'_', '').replace('.ogg', ''))
                        cmd_list.append(cmd)
                cmd_list.sort()
                user[lang] = cmd_list
        users.append(user)
    user_iterator.set_description('Analizing')

columns_description = ['user', 'ita', 'eng']
with open(OUTPUT_PATH, 'w') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames = columns_description)
    writer.writeheader()
    writer.writerows(users)