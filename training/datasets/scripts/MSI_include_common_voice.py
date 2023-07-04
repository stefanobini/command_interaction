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

LANGs = ["eng", "ita"]
DST_NAME = "MSI"
SRC_PATH = os.path.join("datasets", "mozilla_common_voices")
DST_PATH = os.path.join("datasets", DST_NAME, "rejects")
THRESH = True
# N_SAMPLES = 8000
N_SAMPLES = {
    "eng":600/2,
    "esp":100/2,
    "ita":1000/2
}

for lang in LANGs:
# for lan in ['ita']:
    sample_counter = 0
    src_lan_path = os.path.join(SRC_PATH, lang)
    dst_lan_path = os.path.join(DST_PATH, lang)
    os.makedirs(dst_lan_path, exist_ok=True)
    
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

            if  THRESH and sample_counter > N_SAMPLES[lang]:
                break

            sample_counter += 1
        
        file_iter.set_description('Copying {} samples ({} element copied)'.format(lang.upper(), sample_counter))