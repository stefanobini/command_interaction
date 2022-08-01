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


SRC_DATASET = 'GoogleSpeechCommands'
DST_DATASET = 'FELICE_demo7_phase_I/rejects'
# DST_DATASET = 'FELICE_demo3/rejects'
THRESHOLD = 167

SOUND_LIST = [
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

for lan in ['eng', 'ita']:
    dst_path = os.path.join(DST_DATASET, lan, 'clips')
    sound_iter = tqdm(SOUND_LIST)
    for sound in sound_iter:
        src_sound_path = os.path.join(SRC_DATASET, sound)
        dst_sound_path = os.path.join(dst_path, sound)
        i = 0
        for sample in os.listdir(src_sound_path):
            src_sample_path = os.path.join(src_sound_path, sample)
            dst_final_path = dst_sound_path.replace(sound, sound+'_'+sample)
            
            if '.wav' in src_sample_path:
                shutil.copyfile(src_sample_path, dst_final_path)
            elif '.mp3' in src_sample_path:
                convert_audio(src_sample_path, dst_final_path.replace('.mp3', '.wav'))
            
            if i > THRESHOLD:
                break
            i += 1
        
        sound_iter.set_description('Copying in {} folder: {}'.format(lan.upper(), sound))