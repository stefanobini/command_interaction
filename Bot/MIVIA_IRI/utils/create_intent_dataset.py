import os
from tqdm import tqdm
from pydub import AudioSegment
import re
from intents import INTENTS_DICT, CONVERSION_DICT_MSIEXP0, CONVERSION_DICT_MSI

DATASET_NAME = "MSI"
INPUT_DATABASE = os.path.join("recordings")
OUTPUT_DATASET = os.path.join("..", "..", "training", "datasets", DATASET_NAME)
EXPERIMENTATION = "full" # ["reduced", "full"]
REGEX = "^[0-9]*"
intents = INTENTS_DICT
if EXPERIMENTATION == "reduced":
    if DATASET_NAME == "MSI":
        intents = CONVERSION_DICT_MSI
    else:
        intents = CONVERSION_DICT_MSIEXP0


ds_cmd_path = os.path.join(OUTPUT_DATASET, 'commands')
os.makedirs(ds_cmd_path, exist_ok=True)
ds_rej_path = os.path.join(OUTPUT_DATASET, 'rejects')
os.makedirs(ds_rej_path, exist_ok=True)
user_iterator = tqdm(os.listdir(INPUT_DATABASE))
for user in user_iterator:
    bot_user_path = os.path.join(INPUT_DATABASE, user)
    ds_cmd_user_path = os.path.join(ds_cmd_path, user)
    if os.path.isdir(bot_user_path):
        for lang in os.listdir(bot_user_path):
            bot_lang_path = os.path.join(bot_user_path, lang)
            ds_cmd_lang_path = os.path.join(ds_cmd_user_path, lang)
            ds_rej_lang_path = os.path.join(ds_rej_path, lang)
            if os.path.isdir(bot_lang_path):
                # create folder in dst_user
                os.makedirs(ds_cmd_lang_path, exist_ok=True)
                os.makedirs(ds_rej_lang_path, exist_ok=True)
                for file in os.listdir(bot_lang_path):
                    if '.ogg' in file or ".wav" in file:
                        intent = int(file.split('_')[0])
                        bot_sample_path = os.path.join(bot_lang_path, file)
                        
                        # from .ogg to .wav
                        file_ogg = AudioSegment.from_ogg(bot_sample_path)
                        
                        # Change the ID in the reduced version
                        '''
                        if EXPERIMENTATION == "reduced" and intent in CONVERSION_DICT:
                            intent = CONVERSION_DICT[intent]
                            file = re.sub(REGEX, str(intent), file)
                        '''

                        # for intents of interest
                        if intent in intents:
                            ds_cmd_sample_path = os.path.join(ds_cmd_lang_path, file.replace('.ogg', '.wav'))
                            file_handle = file_ogg.export(ds_cmd_sample_path, format='wav')
                        
                        # for reject cmds
                        else:
                            ds_rej_sample_path = os.path.join(ds_rej_lang_path, "telegram_{}_".format(user)+file.replace('.ogg', '.wav'))
                            file_handle = file_ogg.export(ds_rej_sample_path, format='wav')
    
    user_iterator.set_description('Building dataset')