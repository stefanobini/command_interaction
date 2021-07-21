import os
from tqdm import tqdm
from pydub import AudioSegment
from commands_dict import FULL_DATASET, MOSTRINA_CMDS


SRC_DATABASE_PATH = './saves'
OUT_DATASET_PATH = '../Model_Augmentation/dataset/full_dataset_v1'


ds_cmd_path = os.path.join(OUT_DATASET_PATH, 'commands')
user_iterator = tqdm(os.listdir(SRC_DATABASE_PATH))
for user in user_iterator:
    bot_user_path = os.path.join(SRC_DATABASE_PATH, user)
    ds_cmd_user_path = os.path.join(ds_cmd_path, user)
    if os.path.isdir(bot_user_path):
        
        # create folder in dst_dataset
        if not os.path.isdir(ds_cmd_user_path):
            os.mkdir(ds_cmd_user_path)
        for lang in os.listdir(bot_user_path):
            bot_lang_path = os.path.join(bot_user_path, lang)
            ds_cmd_lang_path = os.path.join(ds_cmd_user_path, lang)
            if os.path.isdir(bot_lang_path):
                
                # create folder in dst_user
                if not os.path.isdir(ds_cmd_lang_path):
                    os.mkdir(ds_cmd_lang_path)
                for file in os.listdir(bot_lang_path):
                    if '.ogg' in file:
                        cmd = int(file.replace(lang+'_', '').replace('.ogg', ''))
                        bot_sample_path = os.path.join(bot_lang_path, file)
                        
                        # from .ogg to .wav
                        file_ogg = AudioSegment.from_ogg(bot_sample_path)
                        
                        # for cmds of interest
                        if cmd in FULL_DATASET:
                            ds_cmd_sample_path = os.path.join(ds_cmd_lang_path, file.replace(str(cmd), str(FULL_DATASET[cmd])).replace('.ogg', '.wav'))  # update name for new command
                            file_handle = file_ogg.export(ds_cmd_sample_path, format='wav')
                            
                            # the 18th command (start) is the same for both language, so we duplicate the file
                            if cmd == 18:
                                other_lang = 'ita' if lang == 'eng' else 'eng'
                                ds_other_lang_path = os.path.join(ds_cmd_user_path, other_lang)
                                if not os.path.isdir(ds_other_lang_path):
                                    os.mkdir(ds_other_lang_path)
                                ds_cmd_sample_path = os.path.join(ds_other_lang_path, file.replace(str(cmd), str(FULL_DATASET[cmd]))).replace('.ogg', '.wav')
                                file_handle = file_ogg.export(ds_cmd_sample_path, format='wav')
                        
                        # for italian command: 'mostrina comandi'
                        elif lang == 'ita' and cmd in MOSTRINA_CMDS:
                            file_path = 'mc_' + file.replace(str(cmd), str(MOSTRINA_CMDS[cmd])).replace('.ogg', '.wav')
                            ds_cmd_sample_path_ext = os.path.join(ds_cmd_lang_path, file_path)  # update name for new command
                            file_handle = file_ogg.export(ds_cmd_sample_path_ext, format='wav')
                        
                        # for commands without any change in id
                        else:
                            ds_cmd_sample_path = os.path.join(ds_cmd_lang_path, file.replace('.ogg', '.wav'))
                            file_handle = file_ogg.export(ds_cmd_sample_path, format='wav')
    
    user_iterator.set_description('Building full dataset v1, processing user <{}>'.format(user))