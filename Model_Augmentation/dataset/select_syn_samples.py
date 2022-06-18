import os
import re
import shutil

from tqdm import tqdm
from pydub import AudioSegment


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
CMDS_LIST = CMDS_DICT.keys()

CMDS_EXT_DICT = {
    7:0,
    9:1,
    13:2,
    32:3,
    29:4,
}# 'old_id':'new_id'
CMDS_EXT_LIST = CMDS_EXT_DICT.keys()


def create_folder(base_path, folder):
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
        spk_cmd_path_ext = create_folder(ptf_cmd_path, speaker+'_take')
        spk_rjt_path = create_folder(ptf_rjt_path, speaker)

        for language in os.listdir(spk_syn_path):
            lang_syn_path = os.path.join(spk_syn_path, language)
            
            # create language folder in dataset
            lang_cmd_path = create_folder(spk_cmd_path, language)
            lang_cmd_path_ext = create_folder(spk_cmd_path_ext, language)
            lang_rjt_path = create_folder(spk_rjt_path, language)

            # for "start" command, taht is the same between italian and english commands
            other_lang = 'ita' if language == 'eng' else 'eng'
            other_lang_cmd_path = create_folder(spk_cmd_path_ext, other_lang)
            
            for filee in os.listdir(lang_syn_path):
                file_syn_path = os.path.join(lang_syn_path, filee)
                
                if '.wav' in file_syn_path:
                    regex = '[0-9]{1,2}\.wav$'
                    cmd = int(re.search(pattern=regex, string=file_syn_path).group().split('.')[0])
                    if cmd in CMDS_LIST:
                        file_cmd_path = os.path.join(lang_cmd_path, filee.replace(str(cmd), str(CMDS_DICT[cmd])))
                        shutil.copyfile(file_syn_path, file_cmd_path)
                        if cmd == 18:
                            file_cmd_path = os.path.join(other_lang_cmd_path, filee.replace(str(cmd), str(CMDS_DICT[cmd])))
                            shutil.copyfile(file_syn_path, file_cmd_path)
                    elif cmd in CMDS_EXT_LIST:
                        file_cmd_path_ext = os.path.join(lang_cmd_path_ext, filee.replace(str(cmd), str(CMDS_EXT_DICT[cmd])))
                        shutil.copyfile(file_syn_path, file_cmd_path_ext)
                    else:
                        file_rjt_path_ext = os.path.join(lang_rjt_path, filee)
                        shutil.copyfile(file_syn_path, file_rjt_path_ext)
                
                elif '.mp3' in file_syn_path:
                    file_wav = AudioSegment.from_ogg(file_syn_path) # from .ogg to .wav
                    regex = '[0-9]{1,2}\.mp3$'
                    cmd = int(re.search(pattern=regex, string=file_syn_path).group().split('.')[0])
                    if cmd in CMDS_LIST:
                        file_cmd_path = os.path.join(lang_cmd_path, filee.replace(str(cmd), str(CMDS_DICT[cmd])))
                        file_wav.export(file_cmd_path, format='wav')
                        if cmd == 18:
                            file_cmd_path = os.path.join(other_lang_cmd_path, filee.replace(str(cmd), str(CMDS_DICT[cmd])))
                            file_wav.export(file_cmd_path, format='wav')
                    elif cmd in CMDS_EXT_LIST:
                        file_cmd_path_ext = os.path.join(lang_cmd_path_ext, filee.replace(str(cmd), str(CMDS_EXT_DICT[cmd])))
                        file_wav.export(file_cmd_path_ext, format='wav')
                    else:
                        file_rjt_path_ext = os.path.join(lang_rjt_path, filee)
                        file_wav.export(file_rjt_path_ext, format='wav')
    
    dataset_iterator.set_description('Analizing')