import os
import pandas


DATASET_NAME = "MIVIA_CRF_ISC"
DATASET_FOLDER = os.path.join("datasets", DATASET_NAME)
SAMPLE_PATH = os.path.join(DATASET_FOLDER, "samples")
ANNOTATION_PATH = os.path.join(DATASET_FOLDER, "annotations")
HEADING = ["path", "type", "subtype", "speaker", "command"]
LANGS = ["eng", "ita"]
SUBTYPES= ["clean", "noisy", "all", "test"]
REJECT_LABEL = 31
REJECT_FOLDERS = ["crf_chatter", "fr_chatter"]


data = dict()
for subtype in SUBTYPES:
    data[subtype] = dict()
    for lang in LANGS:
        data[subtype][lang] = {"path":[], "type":[], "subtype":[], "speaker":[], "command":[]}

for speaker in os.listdir(SAMPLE_PATH):
    speaker_path = os.path.join(SAMPLE_PATH, speaker)
    if speaker in REJECT_FOLDERS:
        continue
        for lang in LANGS:
            for sample in os.listdir(speaker_path):
                sample_path = os.path.join("samples", speaker, sample)
                command = REJECT_LABEL
                data["test"][lang]["path"].append(sample_path)
                data["test"][lang]["type"].append("reject")
                data["test"][lang]["subtype"].append(speaker)
                data["test"][lang]["speaker"].append("unknown")
                data["test"][lang]["command"].append(command)

                data["all"][lang]["path"].append(sample_path)
                data["all"][lang]["type"].append("reject")
                data["all"][lang]["subtype"].append(speaker)
                data["all"][lang]["speaker"].append("unknown")
                data["all"][lang]["command"].append(command)

    else:
        for subtype in os.listdir(speaker_path):
            subtype_path = os.path.join(speaker_path, subtype)
            for lang in os.listdir(subtype_path):
                language_path = os.path.join(subtype_path, lang)
                for sample in os.listdir(language_path):
                    sample_path = os.path.join("samples", speaker, subtype, lang, sample)
                    command = int(sample.split('_')[-1].split('.')[0])
                    if command == 31:
                        continue
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

                    data["test"][lang]["path"].append(sample_path)
                    data["test"][lang]["type"].append("command")
                    data["test"][lang]["subtype"].append(subtype)
                    data["test"][lang]["speaker"].append(speaker)
                    data["test"][lang]["command"].append(command)

for lang in LANGS:
    out_path = os.path.join(ANNOTATION_PATH, lang)
    os.makedirs(name=out_path, exist_ok=True)
    for subtype in SUBTYPES:
        out_file = os.path.join(out_path, "{}.csv".format(subtype))
        df = pandas.DataFrame(data=data[subtype][lang], columns=HEADING)
        df.to_csv(path_or_buf=out_file, index=False)