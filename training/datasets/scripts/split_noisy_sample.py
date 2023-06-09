import os
from tqdm import tqdm


DATASET_NAME = "MIVIA_ISC_v1"
PATH:str = os.path.join("datasets", DATASET_NAME, "noises")

def split_audio(path:str) -> None:
    name = path.replace(".wav", "")
    cmd = "ffmpeg -i {}.wav -f segment -segment_time 5 -c copy {}_%03d.wav".format(name, name)
    os.system(cmd)
    cmd = "rm {}".format(path)
    os.system(cmd)

folder_iter = tqdm(os.listdir(PATH))
for folder in folder_iter:
    for file in os.listdir(os.path.join(PATH, folder)):
        if os.path.isdir(os.path.join(PATH, folder, file)):
            for subfile in os.listdir(os.path.join(PATH, folder, file)):
                split_audio(os.path.join(PATH, folder, file, subfile))
        else:
            split_audio(os.path.join(PATH, folder, file))
    folder_iter.set_description("Working on <{}> folder".format(folder))