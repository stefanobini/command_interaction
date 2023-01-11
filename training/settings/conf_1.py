import os
from dotmap import DotMap

settings = DotMap()

'''Input setting'''
settings.input.language:str = "ita"   # ["ita", "eng"]
settings.input.type:str = "mel-spectrogram" # mel-spectrogram
settings.input.sample_rate:int = 16000
settings.input.n_fft:int = 400
settings.input.window:str = "hann"
settings.input.win_sec:float = settings.input.n_fft / settings.input.sample_rate    # default=0.025
settings.input.hop_sec:float = settings.input.win_sec / 2                           # default=0.012
settings.input.win_lenght:int = int(settings.input.win_sec * settings.input.sample_rate)
settings.input.hop_lenght:int = settings.input.win_lenght // 2
settings.input.n_mels:int = 64
settings.input.n_mfcc:int = 64

'''Dataset settings'''
settings.dataset.folder = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1"
settings.dataset.training.annotations.speech:str = os.path.join(settings.dataset.folder, "annotations", settings.input.language, "training.csv")
settings.dataset.training.noise:str = ""
settings.dataset.validation.annotations.speech:str = os.path.join(settings.dataset.folder, "annotations", settings.input.language, "validation.csv")
settings.dataset.validation.noise:str = ""
settings.dataset.testing.annotations.speech:str = os.path.join(settings.dataset.folder, "annotations", settings.input.language, "testing.csv")
settings.dataset.testing.noise:str = ""

'''Model'''
settings.model.resnet8.pooling_size = (4, 3)
settings.model.resnet8.out_channel = 45

'''Training settings'''
settings.training.num_workers:str = 8
settings.training.device:str = "cuda"   # device between ["cpu", "cuda"]
settings.training.gpus:str = "0"        # list of the GPU devices to use
settings.training.max_epochs = 100
settings.training.batch_size:int = 128  # at least 104 for 'ita' and 80 for 'eng' to have in the batch all 31 commands in each batch
settings.training.lr:float = 0.005