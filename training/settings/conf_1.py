import os
from typing import List
from dotmap import DotMap

settings = DotMap()

'''Logger'''
settings.logger.folder:str = "lightning_logs"
settings.logger.name:str = "test"   # name of the experiment

'''Input'''
settings.input.language:str = "ita"   # ["ita", "eng"]
settings.input.type:str = "mfcc" # ["waveform", "melspectrogram", "mfcc"]
settings.input.sample_rate:int = 16000
settings.input.normalize:bool = True
# Spectrogram
settings.input.spec.n_fft:int = 512     # [512, 400]
settings.input.spec.window:str = "hann"
settings.input.spec.win_sec:float = settings.input.spec.n_fft / settings.input.sample_rate    # default=0.025
settings.input.spec.hop_sec:float = settings.input.spec.win_sec / 2                           # default=0.012
settings.input.spec.win_lenght:int = int(settings.input.spec.win_sec * settings.input.sample_rate)
settings.input.spec.hop_lenght:int = settings.input.spec.win_lenght // 2
# Mel
settings.input.mel.n_mels:int = 64  # 40
# MFCC
settings.input.mfcc.n_mfcc:int = 64
settings.input.mfcc.dct_type:int = 2
settings.input.mfcc.norm:str = "ortho"
settings.input.mfcc.log_mels:bool = True    # Default value [False], but NVIDIA NeMo use True

'''Dataset'''
settings.dataset.folder = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/final_dataset"
settings.dataset.speech.training.annotations:str = os.path.join(settings.dataset.folder, "training", "annotations", settings.input.language, "class_training.csv")
settings.dataset.speech.validation.annotations:str = os.path.join(settings.dataset.folder, "validation", "annotations", settings.input.language, "class_validation.csv")
settings.dataset.speech.testing.annotations:str = os.path.join(settings.dataset.folder, "testing", "annotations", settings.input.language, "testing.csv")
settings.dataset.noise.training.annotations:str = os.path.join(settings.dataset.folder, "training", "annotations", "noise", "training.csv")
# settings.dataset.noise.validation.annotations:str = os.path.join(settings.dataset.folder, "annotations", "noise", "validation.csv")
# settings.dataset.noise.testing.annotations:str = os.path.join(settings.dataset.folder, "annotations", "noise", "testing.csv")

'''Model'''
settings.model.network:str = "resnet8"               # ["resnet8", "mobilenetv2", "conformer"]
settings.model.pretrain:bool = False
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
settings.training.num_workers:str = 12
settings.training.accelerator:str = "gpu"                                   # device between ["cpu", "cuda"]
settings.training.devices:int = [0]                                         # list of the GPU devices to use
settings.training.max_epochs = 100
settings.training.min_epochs = 20
settings.training.batch_size:int = 128                                      # at least 104 for 'ita' and 80 for 'eng' to have in the batch all 31 commands in each batch
settings.training.lr.auto_find:bool = False
settings.training.lr.value:float = 0.33                                     # 0.33 - ResNet8,  - MobileNet V2
settings.training.metric_to_track:str = "val_loss"
settings.training.check_val_every_n_epoch:int = 1
settings.training.early_stop.patience:int = 25                              # default=3
settings.training.reduce_lr_on_plateau.patience:int = 10                    # default=10
settings.training.optimizer.mode:str = "min"                                # "min" to minimize the loss, "max" to maximize the loss
settings.training.optimizer.weight_decay:float = 0.001                      # Default 0
settings.training.optimizer.eps:float = 1e-8
settings.training.optimizer.betas:List[float] = [0.9, 0.999]                # Default 0.9, 0.999
settings.training.optimizer.grad_averaging:bool = False
settings.training.optimizer.amsgrad:bool = False

'''Noise & Curriculum Learning'''
settings.noise.min_snr:int = -10
settings.noise.max_snr:int = 40
settings.noise.snr_step:int = 5
settings.noise.descent_ratio:float = 1.0
settings.noise.curriculum_learning.distribution:str = "dynamic_uniform"     # Between ["uniform", "dynamic_uniform", "dynamic_gaussian"]
settings.noise.curriculum_learning.uniform.ab_uniform_step:int = 50         # 50 Uniform CL-PEM v1, Disable in the code for CL-PEM v2
settings.noise.curriculum_learning.gaussian.min_sigma:int = 5               # 5 (Gaussian CL-PEM v2), 25 (Gaussian CL-PEM v1)
settings.noise.curriculum_learning.gaussian.max_sigma:int = 0               # 0 (Gaussian CL-PEM v2), 50 (Gaussian CL-PEM v1)