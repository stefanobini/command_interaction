FILE_PATH = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/validation.csv"
NEW_FILE_PATH = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/final_dataset/validation/annotations/eng/class_validation_0_40.csv"

with open(FILE_PATH, 'r') as r, open(NEW_FILE_PATH, 'w') as o:
    for line in r:
        if line.strip():
            o.write(line)