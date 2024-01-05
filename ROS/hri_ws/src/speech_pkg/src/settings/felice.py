"""Trial with MTL"""
import os
from typing import List
from dotmap import DotMap

settings = DotMap()

settings.name:str = __file__
settings.mode:str = "testing"                                                                                          # ["training", "testing"]
settings.experimentation:str = "FELICE"
settings.task:str = "SCR"                                                                                        # ["SCR", "SI", "SCR_SI"]
settings.tasks:List[str] = ["command"]
info = ""

'''Input'''
settings.input.language:str = "ita"                                                                                 # ["ita", "eng"]
settings.input.type:str = "melspectrogram"                                                                          # ["waveform", "melspectrogram", "mfcc"]
settings.input.sample_rate:int = 16000
settings.input.noise.max_gain:float = 50.
#settings.input.remove_silent:bool = True
# Spectrogram
settings.input.spectrogram.type:str = "db"                                                                          # ["power", "db"] IF YOU CHANGE THIS VALUE CHECH "SPECT_PAD_VALUE and SPECT_PAD_STRiDE" IN dataloaders.py
settings.input.spectrogram.normalize:bool = False
settings.input.spectrogram.n_fft:int = 512                                                                          # [512, 400]
settings.input.spectrogram.window:str = "hann"
settings.input.spectrogram.win_sec:float = settings.input.spectrogram.n_fft / settings.input.sample_rate            # default=0.025
settings.input.spectrogram.hop_sec:float = settings.input.spectrogram.win_sec / 2                                   # default=0.012
settings.input.spectrogram.win_length:int = int(settings.input.spectrogram.win_sec * settings.input.sample_rate)
settings.input.spectrogram.hop_length:int = settings.input.spectrogram.win_length // 2
settings.input.spectrogram.padding.value:float = -80. if settings.input.spectrogram.type == "db" else 0.            #  IF YOU CHANGE THIS VALUE CHECH "SPECT_PAD_VALUE and SPECT_PAD_STRiDE" IN dataloaders.py
settings.input.spectrogram.padding.stride:int = 0                                                                  # silence between two padding patches, in pixel ( IF YOU CHANGE THIS VALUE CHECH "SPECT_PAD_VALUE and SPECT_PAD_STRiDE" IN dataloaders.py)
# Mel
settings.input.mel.n_mels:int = 40                                                                                  # [40, 64]
# MFCC
settings.input.mfcc.n_mfcc:int = 40
settings.input.mfcc.dct_type:int = 2
settings.input.mfcc.norm:str = "ortho"
settings.input.mfcc.log_mels:bool = True                                                                            # Default value [False], but NVIDIA NeMo use True

'''Dataset'''
settings.dataset.folder = os.path.join("datasets", "MIVIA_ISC")
settings.dataset.annotations = os.path.join("datasets", "MTL_scr_sr", "annotations")
settings.dataset.speech.training.annotations:str = os.path.join(settings.dataset.annotations, settings.input.language, "training.csv")
settings.dataset.speech.validation.annotations:str = os.path.join(settings.dataset.annotations, settings.input.language, "validation.csv")
settings.dataset.speech.testing.annotations:str = os.path.join(settings.dataset.annotations, settings.input.language, "testing.csv")
settings.dataset.noise.training.annotations:str = os.path.join(settings.dataset.annotations, "noise", "training.csv")
settings.dataset.knn.annotations = os.path.join("datasets", "MTL_scr_srid", "annotations")
settings.dataset.knn.training.annotations:str = os.path.join(settings.dataset.knn.annotations, settings.input.language, "training.csv")
settings.dataset.knn.testing.annotations:str = os.path.join(settings.dataset.knn.annotations, settings.input.language, "testing.csv")
# settings.dataset.noise.validation.annotations:str = os.path.join(settings.dataset.folder, "annotations", "noise", "validation.csv")
# settings.dataset.noise.testing.annotations:str = os.path.join(settings.dataset.folder, "annotations", "noise", "testing.csv")

'''Model'''
settings.model.network:str = "resnet8"                                      # ["resnet8", "mobilenetv2", "conformer", "HS", "SS"]
settings.model.pretrain:bool = False
settings.model.input.normalize:bool = False
# ResNet8
settings.model.resnet8.pooling_size = (4, 3)                            # Default: (4, 3)
settings.model.resnet8.out_channel = 45                                 # Default: 45
settings.model.resnet8.speaker_embedding_size:int = settings.model.resnet8.out_channel      # beforre was multiplied by 2
settings.model.resnet8.pretrain_path:str = "./pretrained_models/res8_0/model-best.pt.bin"
#settings.model.resnet8.pretrain_path:str = "./lightning_logs/MTL/eng/SCR/resnet8_PEM/checkpoints/epoch=42-step=5160.ckpt"       # SCR, eng
#settings.model.resnet8.pretrain_path:str = "./lightning_logs/MTL/eng/SI/resnet8_PEM/checkpoints/epoch=103-step=12480.ckpt"      # SI, eng
#settings.model.resnet8.pretrain_path:str = "./lightning_logs/MTL/ita/SCR/resnet8_PEM/checkpoints/epoch=60-step=6771.ckpt"       # SCR, ita
settings.model.resnet8.pretrain_path:str = "./lightning_logs/MTL/ita/SI/resnet8_PEM/checkpoints/epoch=60-step=6771.ckpt"        # SI, ita
# MobileNet V2
settings.model.mobilenetv2.speaker_embedding_size:int = 100
settings.model.mobilenetv2.pretrain_path:str = "./pretrained_models/mobilenetv2_0/model-best.pt.bin"
# settings.model.mobilenetv2.pretrain_path:str = ""
# Conformer
settings.model.conformer.num_heads: int = 4
settings.model.conformer.ffn_dim: int = 128
settings.model.conformer.num_layers: int = 4
settings.model.conformer.depthwise_conv_kernel_size: int = 31
settings.model.conformer.dropout: float = 0.0
settings.model.conformer.use_group_norm: bool = False
settings.model.conformer.convolution_first: bool = False
# Multitask - Hard Sharing, resnet8
#settings.model.hard_sharing.pretrain_path:str = "./lightning_logs/MTL/eng/SCR_SI/HS_PEM_grad_norm_alfa_0o05/checkpoints/epoch=71-step=8640.ckpt"    # eng
settings.model.hard_sharing.pretrain_path:str = "./lightning_logs/MTL/ita/SCR_SI/HS_PEM_grad_norm_alfa_0o05/checkpoints/epoch=122-step=13653.ckpt"  # ita
# Multitask - Soft Sharing, resnet8
#settings.model.soft_sharing.pretrain_path:str = "./lightning_logs/MTL/eng/SCR_SI/SS_PEM_grad_norm_alfa_0o05/checkpoints/epoch=193-step=23280.ckpt"  # eng
settings.model.soft_sharing.pretrain_path:str = "./lightning_logs/MTL/ita/SCR_SI/SS_PEM_grad_norm_alfa_0o05/checkpoints/epoch=95-step=10656.ckpt"   # ita

'''k-Nearest Neighbor'''
settings.knn.n_samples_per_speaker:List[int] = [1, 3, 5, 10, 20]
settings.knn.metric:str = "distance"  # ["similarity", "distance"]
settings.knn.function:str = "euclidean"    # ["cosine", "euclidean"]
settings.knn.cosine_similarity.eps:float = 1e-8
settings.knn.cosine_similarity.dim:int = 0
settings.knn.plotting:bool = False  # If <True> plot the train example on two dimension through T-SNE reduction

'''Training'''
settings.training.reject_percentage:float = 0.5
settings.training.num_workers:str = 12
settings.training.accelerator:str = "gpu"                                   # device between ["cpu", "cuda"]
settings.training.device:str = 0                                        # list of the GPU devices to use
settings.training.max_epochs:int = -1
settings.training.min_epochs:int = 1
settings.training.batch_size:int = 128                                      # at least 104 for 'ita' and 80 for 'eng' to have in the batch all 31 commands in each batch
settings.training.lr.auto_find:bool = False
settings.training.lr.value:float = 0.01                                     # 0.33 - ResNet8,  - MobileNet V2
settings.training.checkpoint.metric_to_track:str = "val_loss"
settings.training.checkpoint.save_top_k:int = 1
settings.training.check_val_every_n_epoch:int = 1
settings.training.early_stop.patience:int = 8                              # default=3
settings.training.reduce_lr_on_plateau.patience:int = 5                     # default=10
settings.training.optimizer.mode:str = "min"                                # "min" to minimize the loss, "max" to maximize the loss
settings.training.optimizer.weight_decay:float = 0.0001                      # Default 0
settings.training.optimizer.eps:float = settings.training.lr.value * 1e-3
settings.training.optimizer.betas:List[float] = [0.9, 0.999]                # Default 0.9, 0.999
settings.training.optimizer.grad_averaging:bool = False
settings.training.optimizer.amsgrad:bool = False
settings.training.loss.type:str = "equal_weights"                               # ["grad_norm", "equal_weights"]
settings.training.loss.grad_norm.alpha:float = 0.05                         # Default = 0.12. For task with different level of complexity ah higher value of alpha should be used to enforce the stronger training rate balancing

'''Noise & Curriculum Learning'''
settings.noise.min_snr:int = 40                                              # [-10, 20]
settings.noise.max_snr:int = 40
settings.noise.snr_step:int = 5
settings.noise.descent_ratio:float = 1.0
settings.noise.curriculum_learning.epoch_saturation_time:int = 50
settings.noise.curriculum_learning.distribution:str = "GaussCL_PEM_v2"                 # Between ["PEM", "UniCL_PEM_v1", "UniCL_PEM_v2", "GaussCL_PEM_v1", "GaussCL_PEM_v2"]
settings.noise.curriculum_learning.uniform.step:int = 10
settings.noise.curriculum_learning.gaussian.sigma:int = 10
settings.noise.curriculum_learning.gaussian.max_sigma:int = settings.noise.max_snr - settings.noise.min_snr
settings.noise.curriculum_learning.gaussian.min_sigma:int = settings.noise.curriculum_learning.gaussian.max_sigma / 2

'''Logger'''
settings.logger.folder:str = "experimentations"
settings.logger.name:str = os.path.join(settings.experimentation, settings.input.language, settings.task)       # name of the experiment
settings.logger.version:str = settings.noise.curriculum_learning.distribution

'''Test'''
settings.testing.folder:str = "testing"
settings.testing.n_folds:int = 10
settings.testing.ckpt_path:str = "./lightning_logs/no_reject/02_23_2023-00_45_41/checkpoints/epoch=66-step=2680.ckpt"
settings.testing.results_path:str = None
