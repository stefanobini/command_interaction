import os
from tqdm import tqdm
from pydub import AudioSegment


INPUT_DATABASE = os.path.join("MIVIA_IRI", "recordings")
OUTPUT_DATASET = os.path.join("..", "training", "datasets", "MSI")
INTENTS = [idx for idx in range(23)]

ds_cmd_path = os.path.join(OUTPUT_DATASET, 'commands')
os.makedirs(ds_cmd_path, exist_ok=True)
ds_rej_path = os.path.join(OUTPUT_DATASET, 'rejects')
os.makedirs(ds_rej_path, exist_ok=True)
user_iterator = tqdm(os.listdir(INPUT_DATABASE))
for user in user_iterator:
    bot_user_path = os.path.join(INPUT_DATABASE, user)
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
                        intent = int(file.split('_')[0])
                        bot_sample_path = os.path.join(bot_lang_path, file)
                        
                        # from .ogg to .wav
                        file_ogg = AudioSegment.from_ogg(bot_sample_path)
                        
                        # for intents of interest
                        if intent in INTENTS:
                            ds_cmd_sample_path = os.path.join(ds_cmd_lang_path, file.replace('.ogg', '.wav'))  # update name for new command
                            file_handle = file_ogg.export(ds_cmd_sample_path, format='wav')
                        
                        # for reject cmds
                        else:
                            ds_rej_sample_path = os.path.join(ds_rej_lang_path, file.replace(lang, user+'_'+lang).replace('.ogg', '.wav'))
                            file_handle = file_ogg.export(ds_rej_sample_path, format='wav')
    
    user_iterator.set_description('Building dataset')