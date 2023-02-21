import os
import shutil
import subprocess
from sympy import N

# from pydub import AudioSegment
from tqdm import tqdm


def convert_audio(input_audio, output_audio):
    # cmd = [self.exe_path, "-y", "-i", input_audio, output_audio]
    cmd = ['ffmpeg', "-y", "-i", input_audio, output_audio]
    process = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if process.returncode != 0:
        raise Exception("Conversion error for file:", input_audio)


SRC_PATH = 'FELICE_demo7/reject_plus_common_voice'
# DST_PATH = 'FELICE_demo7_extended/rejects'
# DST_PATH = 'FELICE_demo3/rejects'
DST_PATH = "full_dataset_v1/rejects"
THRESH = True
# N_SAMPLES = 8000
N_SAMPLES = 10000

for lan in ['eng', 'ita']:
# for lan in ['ita']:
    sample_counter = 0
    src_lan_path = os.path.join(SRC_PATH, lan, 'clips')
    dst_lan_path = os.path.join(DST_PATH, lan, 'clips')
    
    file_iter = tqdm(os.listdir(src_lan_path))
    for file in file_iter:
        if 'common' in file:
            src_sample_path = os.path.join(src_lan_path, file)
            dst_sample_path = os.path.join(dst_lan_path, file)

            if '.wav' in src_sample_path:
                shutil.copyfile(src_sample_path, dst_sample_path)
            elif '.mp3' in src_sample_path:
                convert_audio(src_sample_path, dst_sample_path.replace('.mp3', '.wav'))
                # file_wav = AudioSegment.from_mp3(src_sample_path) # from .mp3 to .wav
                # file_wav.export(dst_sample_path.replace('.mp3', '.wav'), format='wav')

            if  THRESH and sample_counter > N_SAMPLES:
                break

            sample_counter += 1
        
        file_iter.set_description('Copying {} samples ({} element copied)'.format(lan.upper(), sample_counter))