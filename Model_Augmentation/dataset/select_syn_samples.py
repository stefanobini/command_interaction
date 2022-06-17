import os
from tqdm import tqdm
from pydub import AudioSegment
import re


SRC_SYN_DATA_PATH = 'Dataset_synth/'
DST_CMD_DATA_PATH = 'FELICE_demo7_extended/commands/'
DST_RJT_DATA_PATH = 'FELICE_demo7_extended/rejects/'


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

# azure:            _{cmd}.wav
# ibm:              _{cmd}.wav
# nemo:             _cmd{cmd}.wav
# vocalware:        _cmd{cmd}.mp3
# google:           _cmd{cmd}.mp3
# naturlreades:     _{cmd}.mp3
# naturaltts:       _{cmd}.mp3
# polly:            _{cmd}.mp3

# platform
## speaker
### language

def create_folder(base_path, folder)
    ptf_cmd_path = os.path.join(base_path, folder)
        if not os.path.isdir(ptf_cmd_path):
            os.mkdir(ptf_cmd_path)
    return ptf_cmd_path


dataset_iterator = tqdm(os.listdir(SRC_SYN_DATA_PATH))
for platform in dataset_iterator:
    ptf_syn_path = os.path.join(SRC_SYN_DATA_PATH, platform)

    # create platform folder in dataset
    ptf_cmd_path = create_folder(DST_CMD_DATA_PATH, platform)
    ptf_rjt_path = create_folder(DST_RJT_DATA_PATH, platform)

    for speaker in os.listdir(ptf_syn_path):
        spk_syn_path = os.path.join(ptf_syn_path, speaker)

        # create speaker folder in dataset
        spk_cmd_path = create_folder(ptf_cmd_path, speaker)
        spk_rjt_path = create_folder(ptf_rjt_path, speaker)

        for language in os.listdir(spk_syn_path):
            lang_syn_path = os.path.join(spk_syn_path, language)
            
            # create language folder in dataset
            spk_cmd_path = create_folder(ptf_cmd_path, speaker)
            spk_rjt_path = create_folder(ptf_rjt_path, speaker)
            
            for filee in os.listdir(lang_syn_path):
                file_syn_path = os.path.join(lang_syn_path, filee)
                
                if '.wav' in file_syn_path:
                    re.

                elif '.mp3' in file_syn_path:
                    
