"""
python3 datasets/utils/FELICE_add_previous_cmds.py
"""
import os
from tqdm import tqdm


DEMO = "7"
IN_DATASETS = {"BOT":os.path.join("datasets", "MIVIA_ISC_v2", "commands"), "CRF":os.path.join("datasets", "recordings")}
OUT_DATASET = os.path.join("datasets", "FELICE", "demo"+DEMO, "commands")

SOURCES = ["BOT", "CRF"]
CONVERSION_DEMOFULL:dict = {
    "BOT":{
        0:0,
        1:1,
        2:2,
        3:3,
        4:4,
        5:5,
        6:6,
        7:7,
        16:12,
        17:13
    },
    "CRF":{
        0:0,
        1:1,
        2:2,
        3:3,
        4:4,
        5:5,
        10:6,
        11:7,
        12:8,
        13:9,
        14:10,
        15:11,
        16:12,
        17:13,
        18:14,
        19:15,
        20:16,
        21:17,
        22:18,
        23:19,
        24:20,
        25:21
    }
}
CONVERSION_DEMO3:dict = {
    "BOT":{
        0:0,
        1:1,
        2:2,
        3:3,
        4:4,
        5:5
    },
    "CRF":{
        0:0,
        1:1,
        2:2,
        3:3,
        4:4,
        5:5
    }
}
CONVERSION_DEMO7:dict = {
    "BOT":{
        6:6,
        7:7,
        16:12,
        17:13
    },
    "CRF":{
        10:0,
        11:1,
        12:2,
        13:3,
        14:4,
        15:5,
        16:6,
        17:7,
        18:8,
        19:9,
        20:10,
        21:11,
        22:12,
        23:13,
        24:14,
        25:15
    }
}
if DEMO == str(3):
    CONVERSION = CONVERSION_DEMO3
elif DEMO == str(7):
    CONVERSION = CONVERSION_DEMO7
elif DEMO == "full":
    CONVERSION = CONVERSION_DEMOFULL

for source in SOURCES:
    speaker_iter = tqdm(os.listdir(path=IN_DATASETS[source]))
    for speaker in speaker_iter:
        in_speaker_path = os.path.join(IN_DATASETS[source], speaker)
        out_speaker_path = os.path.join(OUT_DATASET, source+'_'+speaker)
        os.makedirs(name=out_speaker_path, exist_ok=True)
        speaker_has_not_samples = True
        for lang in os.listdir(in_speaker_path):
            if ".txt" not in lang:  # is a folder
                in_lang_path = os.path.join(in_speaker_path, lang)
                out_lang_path = os.path.join(out_speaker_path, lang)
                os.makedirs(name=out_lang_path, exist_ok=True)
                for sample in os.listdir(in_lang_path):
                    command = int(sample.split('.')[0].split('_')[-1])   # take the command's ID
                    if command in CONVERSION[source]:
                        speaker_has_not_samples = False
                        filename = sample.replace(str(command), str(CONVERSION[source][command]))
                        in_sample_path = os.path.join(in_lang_path, sample)
                        out_sample_path = os.path.join(out_lang_path, filename)
                        shell_command = "cp -n {} {}".format(in_sample_path, out_sample_path)
                        os.system(command=shell_command)
        if speaker_has_not_samples:
            shell_command = "rm -r {}".format(out_speaker_path)
            os.system(command=shell_command)
        speaker_iter.set_description("Working on <{}> samples".format(source))