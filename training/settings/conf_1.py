import os
from dotmap import DotMap

settings = DotMap()

'''Logger'''
settings.logger.folder:str = "lightning_logs"
settings.logger.name:str = "test"   # name of the experiment

'''Input'''
settings.input.language:str = "ita"   # ["ita", "eng"]
settings.input.type:str = "mel-spectrogram" # ["mel-spectrogram", "waveform"]
settings.input.sample_rate:int = 16000
settings.input.n_fft:int = 400
settings.input.window:str = "hann"
settings.input.win_sec:float = settings.input.n_fft / settings.input.sample_rate    # default=0.025
settings.input.hop_sec:float = settings.input.win_sec / 2                           # default=0.012
settings.input.win_lenght:int = int(settings.input.win_sec * settings.input.sample_rate)
settings.input.hop_lenght:int = settings.input.win_lenght // 2
settings.input.n_mels:int = 64
settings.input.n_mfcc:int = 64

'''Dataset'''
settings.dataset.folder = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1"
settings.dataset.speech.training.annotations:str = os.path.join(settings.dataset.folder, "annotations", settings.input.language, "training.csv")
settings.dataset.speech.validation.annotations:str = os.path.join(settings.dataset.folder, "annotations", settings.input.language, "validation.csv")
settings.dataset.speech.testing.annotations:str = os.path.join(settings.dataset.folder, "annotations", settings.input.language, "testing.csv")
settings.dataset.noise.training.annotations:str = os.path.join(settings.dataset.folder, "annotations", "noise", "training.csv")
settings.dataset.noise.validation.annotations:str = os.path.join(settings.dataset.folder, "annotations", "noise", "validation.csv")
settings.dataset.noise.testing.annotations:str = os.path.join(settings.dataset.folder, "annotations", "noise", "testing.csv")

'''Model'''
settings.model.resnet8.pooling_size = (4, 3)
settings.model.resnet8.out_channel = 45

'''Training'''
settings.training.num_workers:str = 8
settings.training.device:str = "gpu"   # device between ["cpu", "cuda"]
settings.training.gpu:int = 1        # list of the GPU devices to use
settings.training.max_epochs = 100
settings.training.batch_size:int = 128  # at least 104 for 'ita' and 80 for 'eng' to have in the batch all 31 commands in each batch
settings.training.lr:float = 0.7
settings.training.optimization_mode:str = "min" # "min" to minimize the loss, "max" to maximize the loss
settings.training.metric_to_track:str = "val_loss_epoch"
settings.training.check_val_every_n_epoch:int = 1
settings.training.early_stop.patience:int = 12  # previous 7
settings.training.scheduler.patience:int = 5    # previous 3

'''Noise & Curriculum Learning'''
settings.noise.min_snr:int = -10
settings.noise.max_snr:int = 40
settings.noise.snr_step:int = 5
settings.noise.descent_ratio:float = 1.0
settings.noise.curriculum_learning.distribution:str = "uniform"   # Between ["uniform", "gaussian"]
settings.noise.curriculum_learning.gaussian.min_sigma:int = 5     # 5 (Gaussian CL-PEM v2), 25 (Gaussian CL-PEM v1)
settings.noise.curriculum_learning.gaussian.max_sigma:int = 0     # 0 (Gaussian CL-PEM v2), 50 (Gaussian CL-PEM v1)