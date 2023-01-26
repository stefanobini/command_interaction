import os
import json
from tqdm import tqdm
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
from datetime import datetime

import torch
import torchaudio
from pytorch_lightning import loggers
import pytorch_lightning as pl
from pytorch_lightning.profiler import SimpleProfiler, AdvancedProfiler

from utils.dataloaders import MiviaDataset, _custom_collate_fn
from utils.preprocessing import Preprocessing
from settings.conf_1 import settings
from models.resnet8 import ResNet8


if torch.cuda.is_available():
    print(Back.GREEN + "CUDA acceleration available on devices: {}".format(settings.training.gpu))
# Set CUDA device
os.environ["CUDA_VISIBLE_DEVICES"] = str(settings.training.gpu)

pin_memory = True if settings.training.device=="cuda" else False

# Set pytorch seed
pl.seed_everything(5138)

#########################
### BUILD DATALOADERS ###
#########################
train_set = MiviaDataset(subset="training", preprocessing=Preprocessing())
labels = train_set._get_labels()
model = ResNet8(num_labels=len(labels)).cuda()  # Load model
weights = train_set._get_label_weights()        # Computing the label weights to balance the dataloader
train_sampler = torch.utils.data.sampler.WeightedRandomSampler(weights=weights, num_samples=len(train_set))
train_loader = torch.utils.data.DataLoader(dataset=train_set, batch_size=settings.training.batch_size, sampler=train_sampler, num_workers=settings.training.num_workers, collate_fn=_custom_collate_fn, pin_memory=pin_memory)
model.train_dataloader(dataloader=train_loader)

val_sets = list()
val_loaders = dict()
for snr in range(settings.noise.min_snr, settings.noise.max_snr+settings.noise.snr_step, settings.noise.snr_step):
    val_set = MiviaDataset(subset="validation", preprocessing=Preprocessing(snr=snr))
    val_sampler = torch.utils.data.sampler.WeightedRandomSampler(weights=weights, num_samples=len(val_set))
    val_sets.append(val_set)
    val_loaders[snr] = torch.utils.data.DataLoader(dataset=val_set, batch_size=settings.training.batch_size, sampler=val_sampler, num_workers=settings.training.num_workers, collate_fn=_custom_collate_fn, pin_memory=pin_memory)
model.val_dataloader(dataloaders=val_loaders)

########################
### BUILD DATALOADER ###
########################

########################
###### TRAIN MODEL #####
########################

# Set log informations
now = datetime.now()
version = now.strftime("%m_%d_%Y-%H_%M_%S")
logger = loggers.TensorBoardLogger(save_dir=settings.logger.folder, name=settings.logger.name, version=version)
info_path = os.path.join(settings.logger.folder, settings.logger.name, version)
profiler = AdvancedProfiler(dirpath=info_path, filename="profiler_summary.txt")

# Set trainer and train model
trainer = pl.Trainer(
    auto_lr_find=False,
    accelerator=settings.training.device,
    gpus=settings.training.gpu,
    logger=logger,
    max_epochs=settings.training.max_epochs,
    # reload_dataloaders_every_epoch=False,       # set True to shuffle the dataloader before start each epoch
    # profiler=profiler,                             # set to True to see how many time was spent from the training process during the training
    # weights_summary="top",                      # set to "full" to see all the weights of each layer of the network
    benchmark=False,                            # set True if the size of the input does not change in order to speed up the training process
    fast_dev_run=False,                          # set True to check each line of the model and training process, it is useful after some change
    # overfit_batch=                              # set to a number of batch on which overfit in order to understand if the training work well
    num_sanity_val_steps=0                      # before training, the sanity validation control performs N validation steps to check if the validation step is working correctly
)
# trainer.tune(model=model, train_dataloaders=train_loader, val_dataloaders=val_loader)
trainer.fit(model=model, train_dataloaders=model.train_loader, val_dataloaders=model.val_loaders)