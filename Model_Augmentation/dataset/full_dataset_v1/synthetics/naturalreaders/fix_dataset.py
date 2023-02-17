import os
from tqdm import tqdm


DATASET_PATH = "neural/ita/"

replace_cmds = {
    16: 21,
    17: 22,
    18: 23,
    19: 24,
    20: 25,
    21: 26,
    22: 27,
    23: 28,
    24: 29,
    25: 30
}
old_cmds = replace_cmds.keys()

remove_cmds = [26, 27]

file_iterator = tqdm(os.listdir(DATASET_PATH))
for file_name in file_iterator:
    if ".mp3" in file_name:
        file_path = os.path.join(DATASET_PATH, file_name)
        cmd = int(file_name.split("_")[2].split(".")[0])
        if cmd in old_cmds:
            new_file_path = file_path.replace("_{}".format(cmd), "_{}".format(replace_cmds[cmd]))
            os.rename(file_path, new_file_path)
            # print(file_path, new_file_path)
    
    file_iterator.set_description("Fixing dataset")