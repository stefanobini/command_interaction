import os
import shutil
import subprocess
import pandas

from tqdm import tqdm
from pydub import AudioSegment


def convert_audio(input_audio, output_audio):
    # cmd = [self.exe_path, "-y", "-i", input_audio, output_audio]
    cmd = ['ffmpeg', "-y", "-i", input_audio, output_audio]
    process = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if process.returncode != 0:
        raise Exception("Conversion error for file:", input_audio)

DST_DATASET_NAME = os.path.join("MIVIA_ISC_v2")
SRC_DATASET = os.path.join("datasets", "google_speech_commands_v1")
DST_DATASET = os.path.join("datasets", DST_DATASET_NAME)
OUTPUT_ANNOTATION_FILE = os.path.join(DST_DATASET, "SCR_google_annotations_LANG.csv")
HEADING = ["path", "type", "subtype", "speaker", "command"]
dst_rjt_dataset = os.path.join(DST_DATASET, "rejects")
dst_cmd_dataset = os.path.join(DST_DATASET, "commands")
os.makedirs(dst_cmd_dataset, exist_ok=True) 
ENG_RJT_DATASET_SIZE = 20000/2
ITA_RJT_DATASET_SIZE = 20000/2
THRESH = True
RJT_LABEL = 31

RJT_DICT = {"eng":[
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
                    'three',
                    'tree',
                    'two',
                    'wow',
                    'yes',
                    'zero'
                ],
            "ita":[
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
                    'three',
                    'tree',
                    'two',
                    'up',
                    'wow',
                    'yes',
                    'zero'
                ]}
CMD_DICT = {"eng":{"go":22, "stop":24},
            "ita":{"stop":24}}

thresholds = dict()
for lang in RJT_DICT:
    thresholds[lang] = int(ENG_RJT_DATASET_SIZE / len(RJT_DICT[lang]))

data = dict()
for lang in RJT_DICT:
    data[lang] = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "command": list()}

rjt_iter = tqdm(RJT_DICT)
for lang in rjt_iter:
    dst_path = os.path.join(dst_rjt_dataset, lang)
    os.makedirs(dst_path, exist_ok=True)
    for sound in RJT_DICT[lang]:
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
            
            if THRESH and i > thresholds[lang]:
                break
            i += 1
    
    df = pandas.DataFrame(data=data[lang], columns=HEADING)
    df.to_csv(path_or_buf=OUTPUT_ANNOTATION_FILE.replace("LANG", lang.upper()), index=False)

    rjt_iter.set_description('Copying REJECTS in {}'.format(lang.upper()))

cmd_iter = tqdm(CMD_DICT)
for lang in cmd_iter:
    for sound in CMD_DICT[lang]:
        src_sound_path = os.path.join(SRC_DATASET, sound)
        for sample in os.listdir(src_sound_path):
            src_sample_path = os.path.join(src_sound_path, sample)
            speaker = sample.split('_')[0]
            dst_speaker_path = os.path.join(dst_cmd_dataset, speaker, lang)
            os.makedirs(dst_speaker_path, exist_ok=True)
            dst_final_path = os.path.join(dst_speaker_path, lang+'_'+str(CMD_DICT[lang][sound])+'_'+sample.split('_')[-1].split('.')[0]+".wav")

            """
            path = os.path.join("commands", speaker, lang, lang+'_'+str(CMD_DICT[lang][sound])+'_'+sample.split('_')[-1].split('.')[0]+".wav")
            data[lang]["path"].append(path)
            data[lang]["type"].append("command")
            data[lang]["subtype"].append("google")
            data[lang]["speaker"].append(speaker)
            data[lang]["command"].append(CMD_DICT[lang][sound])
            """
            
            if '.wav' in src_sample_path:
                shutil.copyfile(src_sample_path, dst_final_path)
            elif '.mp3' in src_sample_path:
                convert_audio(src_sample_path, dst_final_path.replace('.mp3', '.wav'))
    
    rjt_iter.set_description('Copying COMMANDS in {}'.format(lang.upper()))

    """
    df = pandas.DataFrame(data=data[lang], columns=HEADING)
    df.to_csv(path_or_buf=OUTPUT_ANNOTATION_FILE.replace("LANG", lang.upper()), index=False)
    """