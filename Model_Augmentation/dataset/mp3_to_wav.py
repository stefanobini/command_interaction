import os

from tqdm import tqdm
from pydub import AudioSegment

DATASET_PATH = 'FELICE_phase_II/synthetics/polly/neural/eng'

dataset_iter = tqdm(os.listdir(DATASET_PATH))
for file in dataset_iter:
    if '.mp3' in file:
        file_path = os.path.join(DATASET_PATH, file)
        # print(file_path)
        file_wav = AudioSegment.from_mp3(file_path) # from .mp3 to .wav
        file_wav.export(file_path.replace('.mp3', '.wav'), format='wav')

        os.remove(file_path)

    dataset_iter.set_description('Converting files')