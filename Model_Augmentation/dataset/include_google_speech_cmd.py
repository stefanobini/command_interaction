import os
import shutil

from tqdm import tqdm
from pydub import AudioSegment


SRC_DATASET = 'GoogleSpeechCommands'
DST_DATASET = 'FELICE_demo7_extended/rejects'
THRESHOLD = 50

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

dst_path = os.path.join(DST_DATASET, 'eng')
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
            file_wav = AudioSegment.from_mp3(src_sample_path) # from .mp3 to .wav
            file_wav.export(dst_final_path, format='wav')      
        
        if i > THRESHOLD:
            break
        i += 1
    
    sound_iter.set_description('Copying in ENG folder: {}'.format(sound))

dst_path = os.path.join(DST_DATASET, 'ita')
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
            file_wav = AudioSegment.from_mp3(src_sample_path) # from .mp3 to .wav
            file_wav.export(dst_final_path, format='wav')      
        
        if i > THRESHOLD:
            break
        i += 1
    
    sound_iter.set_description('Copying in ITA folder: {}'.format(sound))