from dotmap import DotMap

settings = DotMap()

'''Dataset settings'''
settings.dataset.folder = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1"
settings.dataset.training.sample = ""
settings.dataset.training.noise = ""
settings.dataset.validation.sample = ""
settings.dataset.validation.noise = ""
settings.dataset.test.sample = ""
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