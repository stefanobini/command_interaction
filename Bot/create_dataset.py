import os
from tqdm import tqdm
from pydub import AudioSegment


DATASET_PATH = './saves'
OUTPUT_PATH = '../Model_Augmentation/dataset/FELICE_demo7_extended'
'''CMDS_DICT = {
    6:6,
    8:8,
    12:12,
    31:16,
    28:18,
    30:20,
    18:23
}# 'old_id':'new_id'
'''
CMDS_DICT = {
    6:0,
    8:1,
    12:2,
    31:3,
    28:4,
    30:5,
    18:6
}# 'old_id':'new_id'
CMDS_DICT_EXT = {
    7:0,
    9:1,
    13:2,
    32:3,
    29:4,
}# 'old_id':'new_id'
#'''
CMDS_LIST = CMDS_DICT.keys()
CMDS_EXT_LIST = CMDS_DICT_EXT.keys()    # for command that begin with 'Take'

ds_cmd_path = os.path.join(OUTPUT_PATH, 'commands')
ds_rej_path = os.path.join(OUTPUT_PATH, 'rejects')
user_iterator = tqdm(os.listdir(DATASET_PATH))
for user in user_iterator:
    bot_user_path = os.path.join(DATASET_PATH, user)
    ds_cmd_user_path = os.path.join(ds_cmd_path, user)
    ds_cmd_user_path_ext = os.path.join(ds_cmd_path, user+'_take')
    if os.path.isdir(bot_user_path):
        # create folder in dst_dataset
        if not os.path.isdir(ds_cmd_user_path):
            os.mkdir(ds_cmd_user_path)
        if not os.path.isdir(ds_cmd_user_path_ext):
            os.mkdir(ds_cmd_user_path_ext)
        for lang in os.listdir(bot_user_path):
            bot_lang_path = os.path.join(bot_user_path, lang)
            ds_cmd_lang_path = os.path.join(ds_cmd_user_path, lang)
            ds_cmd_lang_path_ext = os.path.join(ds_cmd_user_path_ext, lang)
            ds_rej_lang_path = os.path.join(ds_rej_path, lang)
            if os.path.isdir(bot_lang_path):
                # create folder in dst_user
                if not os.path.isdir(ds_cmd_lang_path):
                    os.mkdir(ds_cmd_lang_path)
                if not os.path.isdir(ds_cmd_lang_path_ext):
                    os.mkdir(ds_cmd_lang_path_ext)
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
                            # the 18th command (start) is the same for both language, so we duplicate the file
                            if cmd == 18:
                                other_lang = 'ita' if lang == 'eng' else 'eng'
                                ds_other_lang_path = os.path.join(ds_cmd_user_path, other_lang)
                                if not os.path.isdir(ds_other_lang_path):
                                    os.mkdir(ds_other_lang_path)
                                ds_cmd_sample_path = os.path.join(ds_other_lang_path, file.replace(str(cmd), str(CMDS_DICT[cmd]))).replace('.ogg', '.wav')
                                file_handle = file_ogg.export(ds_cmd_sample_path, format='wav')
                        elif cmd in CMDS_EXT_LIST:
                            ds_cmd_sample_path_ext = os.path.join(ds_cmd_lang_path_ext, file.replace(str(cmd), str(CMDS_DICT_EXT[cmd])).replace('.ogg', '.wav'))  # update name for new command
                            file_handle = file_ogg.export(ds_cmd_sample_path_ext, format='wav')
                        else:
                            ds_rej_sample_path = os.path.join(ds_rej_lang_path, file.replace(lang, user+'_'+lang).replace('.ogg', '.wav'))
                            file_handle = file_ogg.export(ds_rej_sample_path, format='wav')
    
    user_iterator.set_description('Analizing')