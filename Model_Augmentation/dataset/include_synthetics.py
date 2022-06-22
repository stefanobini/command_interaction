import os
import re
import shutil

from tqdm import tqdm
from pydub import AudioSegment
from commands_dict import DEMO7_CMDS_DICT, DEMO7_CMDS_DICT_EXT, DEMO3_CMDS_DICT


SRC_SYN_DATA_PATH = 'Dataset_synth/'
DEMO7_DST_CMD_DATA_PATH = 'FELICE_demo7_extended/synthetics/'
DEMO7_DST_RJT_DATA_PATH = 'FELICE_demo7_extended/rejects/'
DEMO3_DST_CMD_DATA_PATH = 'FELICE_demo3/synthetics/'
DEMO3_DST_RJT_DATA_PATH = 'FELICE_demo3/rejects/'


DEMO7_CMDS_LIST = DEMO7_CMDS_DICT.keys()
DEMO7_CMDS_EXT_LIST = DEMO7_CMDS_DICT_EXT.keys()    # for command that begin with 'Take'
DEMO3_CMDS_LIST = DEMO3_CMDS_DICT.keys()


def create_folder(base_path, folder):
    ptf_cmd_path = os.path.join(base_path, folder)
    if not os.path.isdir(ptf_cmd_path):
        os.mkdir(ptf_cmd_path)
    return ptf_cmd_path

'''
dataset_iterator = tqdm(os.listdir(SRC_SYN_DATA_PATH))
for platform in dataset_iterator:
    if '.' not in platform:
        ptf_syn_path = os.path.join(SRC_SYN_DATA_PATH, platform)

        # create platform folder in dataset
        ptf_cmd_path = create_folder(DEMO7_DST_CMD_DATA_PATH, platform)
        #ptf_rjt_path = create_folder(DST_RJT_DATA_PATH, platform)

        for speaker in os.listdir(ptf_syn_path):
            if '.' not in speaker:
                spk_syn_path = os.path.join(ptf_syn_path, speaker)

                # create speaker folder in dataset
                spk_cmd_path = create_folder(ptf_cmd_path, speaker)
                spk_cmd_path_ext = create_folder(ptf_cmd_path, speaker+'_take')
                #spk_rjt_path = create_folder(ptf_rjt_path, speaker)

                for language in os.listdir(spk_syn_path):
                    if '.' not in language:
                        dataset_iterator.set_description('Building DEMO 7 synthetics dataset, {}/{}/{}'.format(platform, speaker, language))
                        
                        lang_syn_path = os.path.join(spk_syn_path, language)
                        
                        # create language folder in dataset
                        lang_cmd_path = create_folder(spk_cmd_path, language)
                        lang_cmd_path_ext = create_folder(spk_cmd_path_ext, language)
                        lang_rjt_path = create_folder(DEMO7_DST_RJT_DATA_PATH, language)

                        # for "start" command, taht is the same between italian and english commands
                        other_lang = 'ita' if language == 'eng' else 'eng'
                        other_lang_cmd_path = create_folder(spk_cmd_path_ext, other_lang)
                        
                        for filee in os.listdir(lang_syn_path):
                            file_syn_path = os.path.join(lang_syn_path, filee)
                            
                            if '.wav' in file_syn_path:
                                regex = '[0-9]{1,2}\.wav$'
                                cmd = int(re.search(pattern=regex, string=file_syn_path).group().split('.')[0])
                                if cmd in DEMO7_CMDS_LIST:
                                    file_cmd_path = os.path.join(lang_cmd_path, filee.replace(str(cmd), str(DEMO7_CMDS_DICT[cmd])))
                                    shutil.copyfile(file_syn_path, file_cmd_path)
                                    if cmd == 18:
                                        file_cmd_path = os.path.join(other_lang_cmd_path, filee.replace(str(cmd), str(DEMO7_CMDS_DICT[cmd])))
                                        shutil.copyfile(file_syn_path, file_cmd_path)
                                elif cmd in DEMO7_CMDS_EXT_LIST:
                                    file_cmd_path_ext = os.path.join(lang_cmd_path_ext, filee.replace(str(cmd), str(DEMO7_CMDS_DICT_EXT[cmd])))
                                    shutil.copyfile(file_syn_path, file_cmd_path_ext)
                                else:
                                    file_rjt_path_ext = os.path.join(lang_rjt_path, filee)
                                    shutil.copyfile(file_syn_path, file_rjt_path_ext)
                            
                            elif '.mp3' in file_syn_path:
                                file_wav = AudioSegment.from_mp3(file_syn_path) # from .mp3 to .wav
                                regex = '[0-9]{1,2}\.mp3$'
                                cmd = int(re.search(pattern=regex, string=file_syn_path).group().split('.')[0])
                                if cmd in DEMO7_CMDS_LIST:
                                    file_cmd_path = os.path.join(lang_cmd_path, filee.replace(str(cmd), str(DEMO7_CMDS_DICT[cmd])).replace('.mp3', '.wav'))
                                    file_wav.export(file_cmd_path, format='wav')
                                    if cmd == 18:
                                        file_cmd_path = os.path.join(other_lang_cmd_path, filee.replace(str(cmd), str(DEMO7_CMDS_DICT[cmd])).replace('.mp3', '.wav'))
                                        file_wav.export(file_cmd_path, format='wav')
                                elif cmd in DEMO7_CMDS_EXT_LIST:
                                    file_cmd_path_ext = os.path.join(lang_cmd_path_ext, filee.replace(str(cmd), str(DEMO7_CMDS_DICT_EXT[cmd])).replace('.mp3', '.wav'))
                                    file_wav.export(file_cmd_path_ext, format='wav')
                                else:
                                    file_rjt_path_ext = os.path.join(lang_rjt_path, filee.replace('.mp3', '.wav'))
                                    file_wav.export(file_rjt_path_ext, format='wav')
'''

dataset_iterator = tqdm(os.listdir(SRC_SYN_DATA_PATH))
for platform in dataset_iterator:
    if '.' not in platform:
        ptf_syn_path = os.path.join(SRC_SYN_DATA_PATH, platform)

        # create platform folder in dataset
        ptf_cmd_path = create_folder(DEMO3_DST_CMD_DATA_PATH, platform)
        #ptf_rjt_path = create_folder(DST_RJT_DATA_PATH, platform)

        for speaker in os.listdir(ptf_syn_path):
            if '.' not in speaker:
                spk_syn_path = os.path.join(ptf_syn_path, speaker)

                # create speaker folder in dataset
                spk_cmd_path = create_folder(ptf_cmd_path, speaker)
                #spk_rjt_path = create_folder(ptf_rjt_path, speaker)

                for language in os.listdir(spk_syn_path):
                    if '.' not in language:
                        dataset_iterator.set_description('Building DEMO 3 synthetics dataset, {}/{}/{}'.format(platform, speaker, language))
                        
                        lang_syn_path = os.path.join(spk_syn_path, language)
                        
                        # create language folder in dataset
                        lang_cmd_path = create_folder(spk_cmd_path, language)
                        lang_rjt_path = create_folder(DEMO3_DST_RJT_DATA_PATH, language)
                        
                        for filee in os.listdir(lang_syn_path):
                            file_syn_path = os.path.join(lang_syn_path, filee)
                            
                            if '.wav' in file_syn_path:
                                regex = '[0-9]{1,2}\.wav$'
                                cmd = int(re.search(pattern=regex, string=file_syn_path).group().split('.')[0])
                                if cmd in DEMO3_CMDS_LIST:
                                    file_cmd_path = os.path.join(lang_cmd_path, filee.replace(str(cmd), str(DEMO3_CMDS_DICT[cmd])))
                                    shutil.copyfile(file_syn_path, file_cmd_path)
                            
                            elif '.mp3' in file_syn_path:
                                file_wav = AudioSegment.from_mp3(file_syn_path) # from .mp3 to .wav
                                regex = '[0-9]{1,2}\.mp3$'
                                cmd = int(re.search(pattern=regex, string=file_syn_path).group().split('.')[0])
                                if cmd in DEMO3_CMDS_LIST:
                                    file_cmd_path = os.path.join(lang_cmd_path, filee.replace(str(cmd), str(DEMO3_CMDS_DICT[cmd]).replace('.mp3', '.wav')))
                                    file_wav.export(file_cmd_path, format='wav')