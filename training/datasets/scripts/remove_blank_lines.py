FILE_PATH = "./datasets/FELICE/demo7_plus/validation/annotations/ita/validation_.csv"
NEW_FILE_PATH = "./datasets/FELICE/demo7_plus/validation/annotations/ita/validation.csv"
with open(FILE_PATH, 'r') as r, open(NEW_FILE_PATH, 'w') as o:
    for line in r:
        if line.strip():
            o.write(line)