import os
import pandas


DATASET_NAME = "MIVIA_CRF_ISC"
DATASET_FOLDER = os.path.join("datasets", DATASET_NAME)
SAMPLE_PATH = os.path.join(DATASET_FOLDER, "samples")
ANNOTATION_PATH = os.path.join(DATASET_FOLDER, "annotations")
HEADING = ["path", "type", "subtype", "speaker", "command"]
LANGS = ["eng", "ita"]
SUBTYPES= ["clean", "noisy", "all"]


data = dict()
for subtype in SUBTYPES:
    data[subtype] = dict()
    for lang in LANGS:
        data[subtype][lang] = {"path":[], "type":[], "subtype":[], "speaker":[], "command":[]}

for speaker in os.listdir(SAMPLE_PATH):
    speaker_path = os.path.join(SAMPLE_PATH, speaker)
    for subtype in os.listdir(speaker_path):
        subtype_path = os.path.join(speaker_path, subtype)
        for lang in os.listdir(subtype_path):
            language_path = os.path.join(subtype_path, lang)
            for sample in os.listdir(language_path):
                sample_path = os.path.join("sample", speaker, subtype, lang, sample)
                command = int(sample.split('_')[-1].split('.')[0])
                data[subtype][lang]["path"].append(sample_path)
                data[subtype][lang]["type"].append("command")
                data[subtype][lang]["subtype"].append(subtype)
                data[subtype][lang]["speaker"].append(speaker)
                data[subtype][lang]["command"].append(command)

                data["all"][lang]["path"].append(sample_path)
                data["all"][lang]["type"].append("command")
                data["all"][lang]["subtype"].append(subtype)
                data["all"][lang]["speaker"].append(speaker)
                data["all"][lang]["command"].append(command)

for lang in LANGS:
    out_path = os.path.join(ANNOTATION_PATH, lang)
    os.makedirs(name=out_path, exist_ok=True)
    for subtype in SUBTYPES:
        out_file = os.path.join(out_path, "{}.csv".format(subtype))
        df = pandas.DataFrame(data=data[subtype][lang], columns=HEADING)
        df.to_csv(path_or_buf=out_file, index=False)