import os
import pandas
# import shutil
import subprocess
from sympy import N
# from pydub import AudioSegment
from tqdm import tqdm
from intents import INTENTS, EXPLICIT_INTENTS, IMPLICIT_INTENTS


def convert_audio(input_audio, output_audio):
    # cmd = [self.exe_path, "-y", "-i", input_audio, output_audio]
    cmd = ['ffmpeg', "-y", "-i", input_audio, output_audio]
    process = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if process.returncode != 0:
        raise Exception("Conversion error for file:", input_audio)

LANGs = ["eng", "esp", "ita"]
DST_NAME = "MSI"
SRC_PATH = os.path.join("datasets", "mozilla_common_voices")
DST_PATH = os.path.join("datasets", DST_NAME, "rejects")
THRESH = True
# N_SAMPLES = 8000
N_SAMPLES = {
    "eng":1600/2,
    "esp":360/2,
    "ita":2300/2
}
HEADING = ["path", "type", "subtype", "speaker", "intent", "explicit", "implicit"]
OUT_ANNOTATION_PATH = os.path.join(SRC_PATH, "MSI_mozilla_annotations_.csv")

for lang in LANGs:
    data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "intent": list(), "explicit": list(), "implicit": list()}
    sample_counter = 0
    src_lan_path = os.path.join(SRC_PATH, lang)
    dst_lan_path = os.path.join(DST_PATH, lang)
    os.makedirs(dst_lan_path, exist_ok=True)
    
    file_iter = tqdm(os.listdir(src_lan_path))
    for file in file_iter:
        if 'common' in file:
            src_sample_path = os.path.join(src_lan_path, file)
            """ONLY IF YOU WANT COPY THE FILE
            dst_sample_path = os.path.join(dst_lan_path, file)
            
            if '.wav' in src_sample_path:
                shutil.copyfile(src_sample_path, dst_sample_path)
            elif '.mp3' in src_sample_path:
                convert_audio(src_sample_path, dst_sample_path.replace('.mp3', '.wav'))
                # file_wav = AudioSegment.from_mp3(src_sample_path) # from .mp3 to .wav
                # file_wav.export(dst_sample_path.replace('.mp3', '.wav'), format='wav')
            #"""

            data["path"].append(file)
            data["type"].append("reject")
            data["subtype"].append("mozilla")
            data["speaker"].append("unknown")
            data["intent"].append(len(INTENTS)-1)
            data["explicit"].append(len(EXPLICIT_INTENTS[lang])-1)
            data["implicit"].append(0)

            if  THRESH and sample_counter > N_SAMPLES[lang]:
                break

            sample_counter += 1
        
        file_iter.set_description('Copying {} samples ({} element copied)'.format(lang.upper(), sample_counter))
    df = pandas.DataFrame(data=data, columns=HEADING)
    df.to_csv(path_or_buf=OUT_ANNOTATION_PATH.replace(".csv", "{}.csv".format(lang.upper())), index=False)