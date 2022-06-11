import os
import sys
from tqdm import tqdm
from pydub import AudioSegment


DATASET_PATH = './saves'
OUTPUT_PATH = './Dataset/Dataset/FELICE_demo7'
CMDS_DICT = {
    6:6,
    8:8,
    12:12,
    31:16,
    28:18,
    30:20,
    18:23
}# 'old_id':'new_id'
CMDS_LIST = CMDS_DICT.keys()

ds_cmd_path = os.path.join(OUTPUT_PATH, 'commands')
ds_rej_path = os.path.join(OUTPUT_PATH, 'rejects')
user_iterator = tqdm(os.listdir(DATASET_PATH))
for user in user_iterator:
    bot_user_path = os.path.join(DATASET_PATH, user)
    ds_cmd_user_path = os.path.join(ds_cmd_path, user)
    if os.path.isdir(bot_user_path):
        # create folder in dst_dataset
        if not os.path.isdir(ds_cmd_user_path):
            os.mkdir(ds_cmd_user_path)
        for lang in os.listdir(bot_user_path):
            bot_lang_path = os.path.join(bot_user_path, lang)
            ds_cmd_lang_path = os.path.join(ds_cmd_user_path, lang)
            ds_rej_lang_path = os.path.join(ds_rej_path, lang)
            if os.path.isdir(bot_lang_path):
                # create folder in dst_user
                if not os.path.isdir(ds_cmd_lang_path):
                    os.mkdir(ds_cmd_lang_path)
                if not os.path.isdir(ds_rej_lang_path):
                    os.mkdir(ds_rej_lang_path)
                for file in os.listdir(bot_lang_path):
                    if '.ogg' in file:
                        cmd = int(file.replace(lang+'_', '').replace('.ogg', ''))
                        bot_sample_path = os.path.join(bot_lang_path, file)
                        # from .ogg to .wav
                        file_ogg = AudioSegment.from_ogg(bot_sample_path)
                        if cmd in CMDS_LIST:
                            ds_cmd_sample_path = os.path.join(ds_cmd_lang_path, file.replace(str(cmd), str(CMDS_DICT[cmd])).replace('.ogg', '.wav'))  # update name for new command
                            file_handle = file_ogg.export(ds_cmd_sample_path, format='wav')
                        else:
                            ds_rej_sample_path = os.path.join(ds_rej_lang_path, file.replace(lang, user+'_'+lang).replace('.ogg', '.wav'))
                            file_handle = file_ogg.export(ds_rej_sample_path, format='wav')
    user_iterator.set_description('Analizing')