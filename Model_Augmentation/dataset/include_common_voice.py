import os
import shutil

from pydub import AudioSegment
from tqdm import tqdm


SRC_PATH = 'FELICE_demo7/reject_plus_common_voice'
DST_PATH = 'FELICE_demo7_extended/rejects'
N_SAMPLES = 1500

sample_counter = 0
# for lan in ['eng', 'ita']:
for lan in ['ita']:
    src_lan_path = os.path.join(SRC_PATH, lan, 'clips')
    dst_lan_path = os.path.join(DST_PATH, lan, 'clips')
    
    file_iter = tqdm(os.listdir(src_lan_path))
    for file in file_iter:
        src_sample_path = os.path.join(src_lan_path, file)
        dst_sample_path = os.path.join(dst_lan_path, file)

        if '.wav' in src_sample_path:
            shutil.copyfile(src_sample_path, dst_sample_path)
        elif '.mp3' in src_sample_path:
            file_wav = AudioSegment.from_mp3(src_sample_path) # from .mp3 to .wav
            file_wav.export(dst_sample_path.replace('.mp3', '.wav'), format='wav')

        if sample_counter >= N_SAMPLES:
            break

        sample_counter += 1
        
        file_iter.set_description('Copying {} samples'.format(lan.upper()))