import os
import sys
from tqdm import tqdm
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore

import torch
import torchaudio
from pytorch_lightning import loggers
import pytorch_lightning as pl

from utils.dataloaders import MiviaDataset, _custom_collate_fn
from settings.conf_1 import settings
from models.resnet8 import ResNet8


if torch.cuda.is_available():
    print(Back.GREEN + "CUDA acceleration available on devices: {}".format(settings.training.gpus))
# Set CUDA device
os.environ["CUDA_VISIBLE_DEVICES"] = settings.training.gpus

# Set pytorch seed
pl.seed_everything(5138)

########################
### LOAD THE SUBSETS ###
########################
transformation = torchaudio.transforms.MelSpectrogram(sample_rate=settings.input.sample_rate, n_fft=settings.input.n_fft, win_length=settings.input.win_lenght, hop_length=settings.input.hop_lenght, n_mels=settings.input.n_mels)
train_set = MiviaDataset(subset="training", transformation=transformation)
val_set = MiviaDataset(subset="validation", transformation=transformation)
test_set = MiviaDataset(subset="testing", transformation=transformation)
# print("The dataset is splitted in:\n- TRAINING samples:\t{}\n- VALIDATION samples:\t{}\n- TESTING samples:\t{}\n".format(len(train_set), len(val_set),len(test_set)))


########################
### BUILD DATALOADER ###
########################
labels = train_set._get_labels()
# print("Labels list ({}): {}".format(len(labels), labels))

# Computing the label weights to balance the dataloader
weights = train_set._get_label_weights()
# print("The label weights are the followings.\n", weights)

# Building a weighted sampler
pin_memory = True if settings.training.device=="cuda" else False

train_sampler = torch.utils.data.sampler.WeightedRandomSampler(weights=weights, num_samples=len(train_set))
val_sampler = torch.utils.data.sampler.WeightedRandomSampler(weights=weights, num_samples=len(val_set))
test_sampler = torch.utils.data.sampler.WeightedRandomSampler(weights=weights, num_samples=len(test_set))

train_loader = torch.utils.data.DataLoader(dataset=train_set, batch_size=settings.training.batch_size, sampler=train_sampler, num_workers=settings.training.num_workers, collate_fn=_custom_collate_fn, pin_memory=pin_memory)
val_loader = torch.utils.data.DataLoader(dataset=val_set, batch_size=settings.training.batch_size, sampler=val_sampler, num_workers=settings.training.num_workers, collate_fn=_custom_collate_fn, pin_memory=pin_memory)
test_loader = torch.utils.data.DataLoader(dataset=test_set, batch_size=settings.training.batch_size, sampler=test_sampler, num_workers=settings.training.num_workers, collate_fn=_custom_collate_fn, pin_memory=pin_memory)

########################
###### TRAIN MODEL #####
########################
model = ResNet8(num_labels=len(labels)).cuda()

logger = loggers.TensorBoardLogger()
trainer = pl.Trainer(auto_lr_find=True, accelerator="gpu", logger=logger, max_epochs=settings.training.max_epochs)   # auto_scale_batch_size=True
trainer.tune(model=model, train_dataloaders=train_loader, val_dataloaders=val_loader)
trainer.fit(model=model, train_dataloaders=train_loader, val_dataloaders=val_loader)