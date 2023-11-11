"""
python3 datasets/utils/FELICE_include_google.py
"""

import os
import shutil
import subprocess
import pandas
from tqdm import tqdm
from FELICE_cmd_map import N_COMMAND_SAMPLES, DEMO_3, DEMO_7


# To change relating to the dataset
DST_DATASET_NAME = os.path.join("FELICE", "demo7")
RJT_LABEL = len(DEMO_7)
THRESH = True

# Fix parameter
SRC_DATASET = os.path.join("datasets", "google_speech_commands_v1")
DST_DATASET = os.path.join("datasets", DST_DATASET_NAME)
OUTPUT_ANNOTATION_FILE = os.path.join(DST_DATASET, "google_annotations_LANG.csv")
HEADING = ["path", "type", "subtype", "speaker", "command"]
LANGS = ["eng", "ita"]
RJT_DATASET_SIZE = N_COMMAND_SAMPLES/3
RJT_CLASS = ['bed', 'bird', 'cat', 'dog', 'down', 'eight', 'five', 'four', 'go', 'happy',
             'house', 'left', 'marvin', 'nine', 'no', 'off', 'on', 'one', 'right', 'seven', 'sheila', 'six', 'stop',
             'three', 'tree', 'two', 'up', 'wow', 'yes', 'zero']


def convert_audio(input_audio, output_audio):
    # cmd = [self.exe_path, "-y", "-i", input_audio, output_audio]
    cmd = ['ffmpeg', "-y", "-i", input_audio, output_audio]
    process = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if process.returncode != 0:
        raise Exception("Conversion error for file:", input_audio)


thresholds = int(RJT_DATASET_SIZE / len(RJT_CLASS))
dst_rjt_dataset = os.path.join(DST_DATASET, "rejects")
dst_cmd_dataset = os.path.join(DST_DATASET, "commands")
os.makedirs(dst_cmd_dataset, exist_ok=True) 

data = dict()
for lang in LANGS:
    data[lang] = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}

lang_iter = tqdm(LANGS)
for lang in lang_iter:
    dst_path = os.path.join(dst_rjt_dataset, lang)
    os.makedirs(dst_path, exist_ok=True)
    for sound in RJT_CLASS:
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
            
            path = os.path.join("rejects", lang, sound+'_'+sample)
            speaker = sample.split('_')[0]
            data[lang]["path"].append(path)
            data[lang]["type"].append("reject")
            data[lang]["subtype"].append("google")
            data[lang]["speaker"].append(speaker)
            data[lang]["command"].append(RJT_LABEL)
            
            if THRESH and i > thresholds:
                break
            i += 1
    
    df = pandas.DataFrame(data=data[lang], columns=HEADING)
    df.to_csv(path_or_buf=OUTPUT_ANNOTATION_FILE.replace("LANG", lang.upper()), index=False)

    lang_iter.set_description('Copying REJECTS in {}'.format(lang.upper()))