import os
from tqdm import tqdm
from pydub import AudioSegment
from commands_dict import DEMO7_CMDS_DICT, DEMO7_CMDS_DICT_EXT, DEMO7_CMDS_DICT_EXT_2, DEMO3_CMDS_DICT


DATASET_PATH = './saves'
DEMO7_OUTPUT_PATH = '../Model_Augmentation/dataset/FELICE_demo7_extended'
DEMO3_OUTPUT_PATH = '../Model_Augmentation/dataset/FELICE_demo3'


#'''
ds_cmd_path = os.path.join(DEMO7_OUTPUT_PATH, 'commands')
ds_rej_path = os.path.join(DEMO7_OUTPUT_PATH, 'rejects')
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
                        
                        # for cmds of interest
                        if cmd in DEMO7_CMDS_DICT:
                            ds_cmd_sample_path = os.path.join(ds_cmd_lang_path, file.replace(str(cmd), str(DEMO7_CMDS_DICT[cmd])).replace('.ogg', '.wav'))  # update name for new command
                            file_handle = file_ogg.export(ds_cmd_sample_path, format='wav')
                            
                            # the 18th command (start) is the same for both language, so we duplicate the file
                            if cmd == 18:
                                other_lang = 'ita' if lang == 'eng' else 'eng'
                                ds_other_lang_path = os.path.join(ds_cmd_user_path, other_lang)
                                if not os.path.isdir(ds_other_lang_path):
                                    os.mkdir(ds_other_lang_path)
                                ds_cmd_sample_path = os.path.join(ds_other_lang_path, file.replace(str(cmd), str(DEMO7_CMDS_DICT[cmd]))).replace('.ogg', '.wav')
                                file_handle = file_ogg.export(ds_cmd_sample_path, format='wav')
                        
                        # for command that begin with 'Take'
                        elif cmd in DEMO7_CMDS_DICT_EXT:
                            file_path = 'take_' + file.replace(str(cmd), str(DEMO7_CMDS_DICT_EXT[cmd])).replace('.ogg', '.wav')
                            ds_cmd_sample_path_ext = os.path.join(ds_cmd_lang_path, file_path)  # update name for new command
                            file_handle = file_ogg.export(ds_cmd_sample_path_ext, format='wav')
                        
                        # for italian command: 'mostrina comandi'
                        elif lang == 'ita' and cmd in DEMO7_CMDS_DICT_EXT_2:
                            file_path = 'mc_' + file.replace(str(cmd), str(DEMO7_CMDS_DICT_EXT_2[cmd])).replace('.ogg', '.wav')
                            ds_cmd_sample_path_ext = os.path.join(ds_cmd_lang_path, file_path)  # update name for new command
                            file_handle = file_ogg.export(ds_cmd_sample_path_ext, format='wav')
                        
                        # for reject cmds
                        else:
                            ds_rej_sample_path = os.path.join(ds_rej_lang_path, file.replace(lang, user+'_'+lang).replace('.ogg', '.wav'))
                            file_handle = file_ogg.export(ds_rej_sample_path, format='wav')
    
    user_iterator.set_description('Building dataset for DEMO 7')

'''

ds_cmd_path = os.path.join(DEMO3_OUTPUT_PATH, 'commands')
ds_rej_path = os.path.join(DEMO3_OUTPUT_PATH, 'rejects')
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
                ds_rej_lang_path = os.path.join(ds_rej_lang_path, 'clips')
                if not os.path.isdir(ds_rej_lang_path):
                    os.mkdir(ds_rej_lang_path)
                for file in os.listdir(bot_lang_path):
                    if '.ogg' in file:
                        cmd = int(file.replace(lang+'_', '').replace('.ogg', ''))
                        bot_sample_path = os.path.join(bot_lang_path, file)
                        # from .ogg to .wav
                        file_ogg = AudioSegment.from_ogg(bot_sample_path)
                        if cmd in DEMO3_CMDS_LIST:
                            ds_cmd_sample_path = os.path.join(ds_cmd_lang_path, file.replace(str(cmd), str(DEMO3_CMDS_DICT[cmd])).replace('.ogg', '.wav'))  # update name for new command
                            file_handle = file_ogg.export(ds_cmd_sample_path, format='wav')
                        else:
                            ds_rej_sample_path = os.path.join(ds_rej_lang_path, file.replace(lang, user+'_'+lang).replace('.ogg', '.wav'))
                            file_handle = file_ogg.export(ds_rej_sample_path, format='wav')
    
    user_iterator.set_description('Building dataset for DEMO 3')
#'''