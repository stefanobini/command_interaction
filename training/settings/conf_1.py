import os
from dotmap import DotMap

settings = DotMap()

'''Language'''
settings.language = "ita"   # ["ita", "eng"]

'''Dataset settings'''
settings.dataset.folder = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1"
settings.dataset.training.annotations.speech = os.path.join(settings.dataset.folder, "annotations", settings.language, "training.csv")
settings.dataset.training.noise = ""
settings.dataset.validation.annotations.speech = os.path.join(settings.dataset.folder, "annotations", settings.language, "validation.csv")
settings.dataset.validation.noise = ""
settings.dataset.testing.annotations.speech = os.path.join(settings.dataset.folder, "annotations", settings.language, "testing.csv")
settings.dataset.testing.noise = ""

'''Training settings'''
settings.training.num_workers = 8
settings.training.device = "cuda"   # device between ["cpu", "cuda"]
settings.training.gpus = [0]        # list of the GPU devices to use
settings.training.batch_size = 64

'''Input setting'''
settings.input.sample_rate = 16000
settings.input.n_fft = 512
settings.input.window = "hann"
settings.input.win_lenght = 12
settings.input.window_hop = 5
settings.input.n_mels = 64
settings.input.n_mfcc = 64