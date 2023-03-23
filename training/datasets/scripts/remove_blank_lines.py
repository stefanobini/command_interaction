FILE_PATH = "./datasets/MTL_experimentation_0/validation/annotations/eng/validation_snr40_.csv"
NEW_FILE_PATH = "./datasets/MTL_experimentation_0/validation/annotations/eng/validation_snr40.csv"

with open(FILE_PATH, 'r') as r, open(NEW_FILE_PATH, 'w') as o:
    for line in r:
        if line.strip():
            o.write(line)