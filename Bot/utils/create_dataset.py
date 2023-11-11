"""
python3 utils/create_dataset.py
"""
import os
from tqdm import tqdm
from pydub import AudioSegment
from commands_dict import DEMO_3, DEMO_7, DEMO_FULL


DEMO = "demo7"  # ["demo3", "demo7", "demofull"]
DATASET_PATH = os.path.join(".", "saves")
FELICE_DEMO_PATH = os.path.join("..", "training", "datasets", "FELICE", DEMO)
if DEMO == "demo3":
    COMMANDS = DEMO_3
elif DEMO == "demo7":
    COMMANDS = DEMO_7
elif DEMO == "demofull":
    COMMANDS = DEMO_FULL
else:
    print("Demo <{}> is not available".format(DEMO))

ds_cmd_path = os.path.join(FELICE_DEMO_PATH, 'commands')
os.makedirs(ds_cmd_path, exist_ok=True)
ds_rej_path = os.path.join(FELICE_DEMO_PATH, 'rejects')
os.makedirs(ds_rej_path, exist_ok=True)
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
                        if cmd in COMMANDS[lang]:
                            ds_cmd_sample_path = os.path.join(ds_cmd_lang_path, file.replace(str(cmd), str(cmd)).replace('.ogg', '.wav'))  # update name for new command
                            file_handle = file_ogg.export(ds_cmd_sample_path, format='wav')
                        
                        # for reject cmds
                        else:
                            ds_rej_sample_path = os.path.join(ds_rej_lang_path, file.replace(lang, user+'_'+lang).replace('.ogg', '.wav'))
                            file_handle = file_ogg.export(ds_rej_sample_path, format='wav')
    
    user_iterator.set_description('Building dataset for full FFELICE DEMO')