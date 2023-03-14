FILE_PATH = "/mnt/sdc1/sbini/command_interaction/training/datasets/MTL_experimentation/validation/annotations/ita/validation_no_noise_.csv"
NEW_FILE_PATH = "/mnt/sdc1/sbini/command_interaction/training/datasets/MTL_experimentation/validation/annotations/ita/validation_no_noise.csv"

with open(FILE_PATH, 'r') as r, open(NEW_FILE_PATH, 'w') as o:
    for line in r:
        if line.strip():
            o.write(line)