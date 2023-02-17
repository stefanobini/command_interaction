FILE_PATH = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/final_dataset/training/annotations/ita/reduced_class_training_.csv"
NEW_FILE_PATH = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/final_dataset/training/annotations/ita/reduced_class_training.csv"

with open(FILE_PATH, 'r') as r, open(NEW_FILE_PATH, 'w') as o:
    for line in r:
        if line.strip():
            o.write(line)