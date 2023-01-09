import os
from dotmap import DotMap

settings = DotMap()

'''Language'''
settings.language = "ita"   # ["ita", "eng"]

'''Dataset settings'''
settings.dataset.folder = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1"
settings.dataset.train.annotations.speech = os.path.join(settings.dataset.folder, "annotations", settings.language, "train.csv")
settings.dataset.train.noise = ""
settings.dataset.valid.annotations.speech = os.path.join(settings.dataset.folder, "annotations", settings.language, "valid.csv")
settings.dataset.valid.noise = ""
settings.dataset.test.annotations.speech = os.path.join(settings.dataset.folder, "annotations", settings.language, "test.csv")
settings.dataset.test.noise = ""

'''Training settings'''
settings.training

'''Input setting'''
settings.input.sample_rate = 16000
settings.input.n_fft = 512
settings.input.window = "hann"
settings.input.window_lenght = 12
settings.input.window_hop = 5
settings.input.n_mels = 64
settings.input.n_mfcc = 64