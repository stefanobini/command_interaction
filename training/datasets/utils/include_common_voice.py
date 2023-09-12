import os
import shutil
import subprocess
from sympy import N
import pandas

# from pydub import AudioSegment
from tqdm import tqdm


def convert_audio(input_audio, output_audio):
    # cmd = [self.exe_path, "-y", "-i", input_audio, output_audio]
    cmd = ['ffmpeg', "-y", "-i", input_audio, output_audio]
    process = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if process.returncode != 0:
        raise Exception("Conversion error for file:", input_audio)

DST_NAME = "MIVIA_ISC_v2"
SRC_PATH = os.path.join("datasets", "mozilla_common_voices")
DST_PATH = os.path.join("datasets", DST_NAME)
OUTPUT_ANNOTATION_FILE = os.path.join(DST_PATH, "SCR_mozilla_annotations_LANG.csv")
HEADING = ["path", "type", "subtype", "speaker", "command"]
THRESH = True
# N_SAMPLES = 8000
N_SAMPLES = 10000
RJT_LABEL = 31


for lang in ['eng', 'ita']:
    data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}
    sample_counter = 0
    src_lan_path = os.path.join(SRC_PATH, lang)
    dst_lan_path = os.path.join(DST_PATH, "rejects", lang)
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

            path = os.path.join("rejects", lang, file)
            data["path"].append(path)
            data["type"].append("reject")
            data["subtype"].append("mozilla")
            data["speaker"].append("unknown")
            data["command"].append(RJT_LABEL)
            
            if  THRESH and sample_counter > N_SAMPLES:
                break

            sample_counter += 1
        
        file_iter.set_description('Copying {} samples ({} element copied)'.format(lang.upper(), sample_counter))
    
    df = pandas.DataFrame(data=data, columns=HEADING)
    df.to_csv(path_or_buf=OUTPUT_ANNOTATION_FILE.replace("LANG", lang.upper()), index=False)