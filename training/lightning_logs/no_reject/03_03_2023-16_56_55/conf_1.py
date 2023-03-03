"""UniCL_PEM_v2 with ENG commands"""
import os
from typing import List
from dotmap import DotMap

settings = DotMap()

settings.name:str = "conf_1.py"
settings.mode:str = "testing"                                                                                          # ["training", "test"]

'''Logger'''
settings.logger.folder:str = "lightning_logs"
settings.logger.name:str = "no_reject"                                                                              # name of the experiment

'''Input'''
settings.input.language:str = "eng"                                                                                 # ["ita", "eng"]
settings.input.type:str = "melspectrogram"                                                                          # ["waveform", "melspectrogram", "mfcc"]
settings.input.sample_rate:int = 16000
settings.input.noise.max_gain:float = 50.
#settings.input.remove_silent:bool = True
# Spectrogram
settings.input.spectrogram.type:str = "db"                                                                          # ["power", "db"]
settings.input.spectrogram.normalize:bool = False
settings.input.spectrogram.n_fft:int = 512                                                                          # [512, 400]
settings.input.spectrogram.window:str = "hann"
settings.input.spectrogram.win_sec:float = settings.input.spectrogram.n_fft / settings.input.sample_rate            # default=0.025
settings.input.spectrogram.hop_sec:float = settings.input.spectrogram.win_sec / 2                                   # default=0.012
settings.input.spectrogram.win_length:int = int(settings.input.spectrogram.win_sec * settings.input.sample_rate)
settings.input.spectrogram.hop_length:int = settings.input.spectrogram.win_length // 2
settings.input.spectrogram.padding.value:float = -80. if settings.input.spectrogram.type == "db" else 0.
settings.input.spectrogram.padding.stride:int = 0                                                                  # silence between two padding patches, in pixel
# Mel
settings.input.mel.n_mels:int = 40                                                                                  # [40, 64]
# MFCC
settings.input.mfcc.n_mfcc:int = 40
settings.input.mfcc.dct_type:int = 2
settings.input.mfcc.norm:str = "ortho"
settings.input.mfcc.log_mels:bool = True                                                                            # Default value [False], but NVIDIA NeMo use True

'''Dataset'''
settings.dataset.folder = "/mnt/sdc1/sbini/command_interaction/training/datasets/final_dataset"
settings.dataset.speech.training.annotations:str = os.path.join(settings.dataset.folder, "training", "annotations", settings.input.language, "training.csv")
settings.dataset.speech.validation.annotations:str = os.path.join(settings.dataset.folder, "validation", "annotations", settings.input.language, "class_validation_0_40.csv")
settings.dataset.speech.testing.annotations:str = os.path.join(settings.dataset.folder, "testing", "annotations", settings.input.language, "class_testing_0_40.csv")
settings.dataset.noise.training.annotations:str = os.path.join(settings.dataset.folder, "training", "annotations", "noise", "training.csv")
# settings.dataset.noise.validation.annotations:str = os.path.join(settings.dataset.folder, "annotations", "noise", "validation.csv")
# settings.dataset.noise.testing.annotations:str = os.path.join(settings.dataset.folder, "annotations", "noise", "testing.csv")

'''Model'''
settings.model.network:str = "resnet8"                                      # ["resnet8", "mobilenetv2", "conformer"]
settings.model.pretrain:bool = False
settings.model.input.normalize:bool = False
# ResNet8
settings.model.resnet8.pooling_size = (4, 3)
settings.model.resnet8.out_channel = 45
settings.model.resnet8.pretrain_path:str = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/pretrained_models/res8_0/model-best.pt.bin"
# MobileNet V2
settings.model.mobilenetv2.pretrain_path:str = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/pretrained_models/mobilenetv2_0/model-best.pt.bin"
# Conformer
settings.model.conformer.num_heads: int = 4
settings.model.conformer.ffn_dim: int = 128
settings.model.conformer.num_layers: int = 4
settings.model.conformer.depthwise_conv_kernel_size: int = 31
settings.model.conformer.dropout: float = 0.0
settings.model.conformer.use_group_norm: bool = False
settings.model.conformer.convolution_first: bool = False

'''Training'''
settings.training.reject_percentage:float = 0.5
settings.training.num_workers:str = 64
settings.training.accelerator:str = "gpu"                                   # device between ["cpu", "cuda"]
settings.training.devices:int = [3]                                         # list of the GPU devices to use
settings.training.max_epochs:int = -1
settings.training.min_epochs:int = 1
settings.training.batch_size:int = 128                                      # at least 104 for 'ita' and 80 for 'eng' to have in the batch all 31 commands in each batch
settings.training.lr.auto_find:bool = False
settings.training.lr.value:float = 0.01                                     # 0.33 - ResNet8,  - MobileNet V2
settings.training.checkpoint.metric_to_track:str = "val_loss"
settings.training.checkpoint.save_top_k:int = 3
settings.training.check_val_every_n_epoch:int = 1
settings.training.early_stop.patience:int = 8                              # default=3
settings.training.reduce_lr_on_plateau.patience:int = 5                     # default=10
settings.training.optimizer.mode:str = "min"                                # "min" to minimize the loss, "max" to maximize the loss
settings.training.optimizer.weight_decay:float = 0.001                      # Default 0
settings.training.optimizer.eps:float = settings.training.lr.value * 1e-2
settings.training.optimizer.betas:List[float] = [0.9, 0.999]                # Default 0.9, 0.999
settings.training.optimizer.grad_averaging:bool = False
settings.training.optimizer.amsgrad:bool = False

'''Noise & Curriculum Learning'''
settings.noise.min_snr:int = 0                                              # [-10, 0]
settings.noise.max_snr:int = 40
settings.noise.snr_step:int = 5
settings.noise.descent_ratio:float = 1.0
settings.noise.curriculum_learning.epoch_saturation_time:int = 50
settings.noise.curriculum_learning.distribution:str = "UniCL_PEM_v2"                 # Between ["PEM", "UniCL_PEM_v1", "UniCL_PEM_v2", "GaussCL_PEM_v1", "GaussCL_PEM_v2"]
settings.noise.curriculum_learning.uniform.step:int = 10
settings.noise.curriculum_learning.gaussian.sigma:int = 10
settings.noise.curriculum_learning.gaussian.max_sigma:int = settings.noise.max_snr - settings.noise.min_snr
settings.noise.curriculum_learning.gaussian.min_sigma:int = settings.noise.curriculum_learning.gaussian.max_sigma / 2

'''Test'''
settings.testing.folder:str = "testing"
settings.testing.n_fold:int = 10
settings.testing.ckpt_path:str = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/lightning_logs/no_reject/02_23_2023-00_45_41/checkpoints/epoch=66-step=2680.ckpt"
settings.testing.results_path:str = None