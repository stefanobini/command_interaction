import os
import shutil
import subprocess

from tqdm import tqdm
from pydub import AudioSegment


def convert_audio(input_audio, output_audio):
    # cmd = [self.exe_path, "-y", "-i", input_audio, output_audio]
    cmd = ['ffmpeg', "-y", "-i", input_audio, output_audio]
    process = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if process.returncode != 0:
        raise Exception("Conversion error for file:", input_audio)

DST_DATASET_NAME = os.path.join("MIVIA_ISC")
SRC_DATASET = os.path.join("datasets", "google_speech_commands_v1")
DST_DATASET = os.path.join("datasets", DST_DATASET_NAME)
dst_rjt_dataset = os.path.join(DST_DATASET, "rejects")
ENG_RJT_DATASET_SIZE = 20000/2
ITA_RJT_DATASET_SIZE = 20000/2
THRESH = True

ENG_RJT_LIST = [
    '_background_noise_',
    'bed',
    'bird',
    'cat',
    'dog',
    'eight',
    'five',
    'four',
    'happy',
    'house',
    'marvin',
    'nine',
    'no',
    'off',
    'on',
    'one',
    'seven',
    'sheila',
    'six',
    'stop',
    'three',
    'tree',
    'two',
    'wow',
    'yes',
    'zero'
]

ITA_RJT_LIST = [
    '_background_noise_',
    'bed',
    'bird',
    'cat',
    'dog',
    'down',
    'eight',
    'five',
    'four',
    'go',
    'happy',
    'house',
    'left',
    'marvin',
    'nine',
    'no',
    'off',
    'on',
    'one',
    'right',
    'seven',
    'sheila',
    'six',
    'stop',
    'three',
    'tree',
    'two',
    'up',
    'wow',
    'yes',
    'zero'
]

ENG_THRESHOLD = int(ENG_RJT_DATASET_SIZE / len(ENG_RJT_LIST))
ITA_THRESHOLD = int(ITA_RJT_DATASET_SIZE / len(ITA_RJT_LIST))

### ENGLISH DATASET ###
lang = "eng"
dst_path = os.path.join(dst_rjt_dataset, lang)
os.makedirs(dst_path, exist_ok=True)

sound_iter = tqdm(ENG_RJT_LIST)
for sound in sound_iter:
    src_sound_path = os.path.join(SRC_DATASET, sound)
    i = 0
    for sample in os.listdir(src_sound_path):
        src_sample_path = os.path.join(src_sound_path, sample)
        dst_filename = sound+'_'+sample
        dst_final_path = os.path.join(dst_path, dst_filename)
        
        if '.wav' in src_sample_path:
            shutil.copyfile(src_sample_path, dst_final_path)
        elif '.mp3' in src_sample_path:
            convert_audio(src_sample_path, dst_final_path.replace('.mp3', '.wav'))
        
        if THRESH and i > ENG_THRESHOLD:
            break
        i += 1
    
    sound_iter.set_description('Copying in {} folder: {}'.format(lang.upper(), sound))
    
### ITALIAN DATASET ###
lang = "ita"
dst_path = os.path.join(dst_rjt_dataset, lang)
os.makedirs(dst_path, exist_ok=True)

sound_iter = tqdm(ITA_RJT_LIST)
for sound in sound_iter:
    src_sound_path = os.path.join(SRC_DATASET, sound)
    i = 0
    for sample in os.listdir(src_sound_path):
        src_sample_path = os.path.join(src_sound_path, sample)
        dst_filename = sound+'_'+sample
        dst_final_path = os.path.join(dst_path, dst_filename)
        
        if '.wav' in src_sample_path:
            shutil.copyfile(src_sample_path, dst_final_path)
        elif '.mp3' in src_sample_path:
            convert_audio(src_sample_path, dst_final_path.replace('.mp3', '.wav'))
        
        if THRESH and i > ITA_THRESHOLD:
            break
        i += 1
    
    sound_iter.set_description('Copying in {} folder: {}'.format(lang.upper(), sound))