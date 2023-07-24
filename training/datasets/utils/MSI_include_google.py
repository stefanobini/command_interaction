import os
import shutil
import subprocess
from tqdm import tqdm
from pydub import AudioSegment
import pandas

from intents import INTENTS, EXPLICIT_INTENTS, IMPLICIT_INTENTS, INTENTS_DICT_MSIEXP1, INTENTS_DICT_MSIEXP0


def convert_audio(input_audio, output_audio):
    # cmd = [self.exe_path, "-y", "-i", input_audio, output_audio]
    cmd = ['ffmpeg', "-y", "-i", input_audio, output_audio]
    process = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if process.returncode != 0:
        raise Exception("Conversion error for file:", input_audio)

DST_DATASET_NAME = "MSIexp1"
SRC_DATASET = os.path.join("datasets", "google_speech_commands_v1")
DST_DATASET = os.path.join("datasets", DST_DATASET_NAME)
dst_rjt_dataset = os.path.join(DST_DATASET, "rejects")
ENG_RJT_DATASET_SIZE = 2400/4
ESP_RJT_DATASET_SIZE = 630/4
ITA_RJT_DATASET_SIZE = 2600/4
THRESH = True
LANGs = ["eng", "esp", "ita"]
CMD_LIST = {
    "eng":["down", "left", "right", "stop"],
    "esp":["stop"],
    "ita":["stop"]
}
RJT_LIST = {
    "eng":[
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
    "esp":[
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
    ],
    "ita":[
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
}
RJT_DATASET_SIZE = {
    "eng":ENG_RJT_DATASET_SIZE,
    "esp":ESP_RJT_DATASET_SIZE,
    "ita":ITA_RJT_DATASET_SIZE
}
HEADING = ["path", "type", "subtype", "speaker", "intent", "explicit", "implicit"]
OUT_ANNOTATION_PATH = os.path.join(DST_DATASET, "google_annotations_.csv")

for lang in LANGs:
    '''Set the reject label'''
    rjt_intents = len(INTENTS)-1
    rjt_exp_intents = INTENTS[rjt_intents]["explicit"][lang][0]["id"]
    rjt_imp_intents = 0
    if "MSIexp0" in DST_DATASET_NAME:
        rjt_intents = len(INTENTS_DICT_MSIEXP0)-1
        rjt_exp_intents = INTENTS_DICT_MSIEXP0[rjt_intents]["explicit"][lang][0]["id"]
    elif "MSIexp1" in DST_DATASET_NAME:
        rjt_intents = len(INTENTS_DICT_MSIEXP1)-1
        rjt_exp_intents = INTENTS_DICT_MSIEXP1[rjt_intents]["explicit"][lang][0]["id"]

    dst_path = os.path.join(dst_rjt_dataset, lang)
    #os.makedirs(dst_path, exist_ok=True)   # ONLY IF YOU WANT ALSO COPY THE FILES
    threshold = int(RJT_DATASET_SIZE[lang] / len(RJT_LIST[lang]))
    data = {"path": list(), "type": list(), "subtype": list(), "speaker": list(), "intent": list(), "explicit": list(), "implicit": list()}

    '''
    sound_iter = tqdm(RJT_LIST[lang])
    for sound in sound_iter:
        src_sound_path = os.path.join(SRC_DATASET, sound)
        i = 0
        for sample in os.listdir(src_sound_path):
            src_sample_path = os.path.join(src_sound_path, sample)
            """ONLY IF YOU WANT ALSO COPY THE FILES
            dst_filename = sound+'_'+sample
            dst_final_path = os.path.join(dst_path, dst_filename)
            #"""
            
            """ ONLY IF YOU WANT ALSO COPY THE FILES
            if '.wav' in src_sample_path:
                shutil.copyfile(src_sample_path, dst_final_path)
            elif '.mp3' in src_sample_path:
                convert_audio(src_sample_path, dst_final_path.replace('.mp3', '.wav'))
            #"""

            data["path"].append(os.path.join(sound, sample))
            data["type"].append("reject")
            data["subtype"].append("google_speech")
            data["speaker"].append(sample.split('_')[0])
            data["intent"].append(rjt_intents)
            data["explicit"].append(rjt_exp_intents)
            data["implicit"].append(rjt_imp_intents)

            if THRESH and i > threshold:
                break
            i += 1
        
        sound_iter.set_description('Copying <{}> samples of COMMAND: {}'.format(lang.upper(), sound))
    '''

    sound_iter = tqdm(RJT_LIST[lang])
    for sound in sound_iter:
        src_sound_path = os.path.join(SRC_DATASET, sound)
        i = 0
        for sample in os.listdir(src_sound_path):
            src_sample_path = os.path.join(src_sound_path, sample)
            """ONLY IF YOU WANT ALSO COPY THE FILES
            dst_filename = sound+'_'+sample
            dst_final_path = os.path.join(dst_path, dst_filename)
            #"""
            
            """ ONLY IF YOU WANT ALSO COPY THE FILES
            if '.wav' in src_sample_path:
                shutil.copyfile(src_sample_path, dst_final_path)
            elif '.mp3' in src_sample_path:
                convert_audio(src_sample_path, dst_final_path.replace('.mp3', '.wav'))
            #"""

            data["path"].append(os.path.join(sound, sample))
            data["type"].append("reject")
            data["subtype"].append("google_speech")
            data["speaker"].append(sample.split('_')[0])
            data["intent"].append(rjt_intents)
            data["explicit"].append(rjt_exp_intents)
            data["implicit"].append(rjt_imp_intents)

            if THRESH and i > threshold:
                break
            i += 1
        
        sound_iter.set_description('Copying <{}> samples of REJECT: {}'.format(lang.upper(), sound))
    df = pandas.DataFrame(data=data, columns=HEADING)
    df.to_csv(path_or_buf=OUT_ANNOTATION_PATH.replace(".csv", "{}.csv".format(lang.upper())), index=False)