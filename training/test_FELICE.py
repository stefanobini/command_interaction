import os
import sys
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
import argparse
import numpy as np

import torch
import pytorch_lightning as pl

from utils.dataloaders import TestingMiviaDataset, _SCR_val_collate_fn, _SI_val_collate_fn, _MT_val_collate_fn

from models.resnet8 import ResNet8_PL
from models.mobilenetv2 import MobileNetV2_PL
from models.conformer import Conformer_PL
from models.hardsharing import HardSharing_PL
# from models.hardsharing_mobilenetv2 import HardSharing_PL
from models.softsharing import SoftSharing_PL
# from models.softsharing_mobilenetv2 import SoftSharing_PL


########################
#     Setting CUDA     #
########################
parser = argparse.ArgumentParser()
parser.add_argument("--configuration", type=str, dest="configuration", required=True, help="Configuration file (e.g., 'conf_1')")
args = parser.parse_args()
args.configuration = "settings.{}".format(args.configuration)
settings = getattr(__import__(args.configuration, fromlist=["settings"]), "settings")
pin_memory = True if settings.training.accelerator=="gpu" else False
print(Back.CYAN + "Loaded <{}> as configuration file.".format(settings.name))

########################
#     Setting Seed     #
########################
pl.seed_everything(220295)

#########################
# Building Dataloaders  #
#########################
N_LABELS = 7
labels_1 = [i for i in range(N_LABELS)]
# print(labels_1, len(labels_1))
# print(labels_2, len(labels_2)) # 44, 85
labels = labels_1
collate_fn = _SCR_val_collate_fn

test_loaders = list()
for fold in range(settings.testing.n_folds):
    test_set = TestingMiviaDataset(settings=settings, fold=fold)
    test_loaders.append(torch.utils.data.DataLoader(dataset=test_set, batch_size=settings.training.batch_size, shuffle=None, num_workers=settings.training.num_workers, collate_fn=collate_fn, pin_memory=pin_memory))


#########################
#    Building Model     #
#########################
assert settings.model.network in ["resnet8"]
ckpt_folder = os.path.join(settings.logger.folder, settings.logger.name, settings.logger.version, "checkpoints")
ckpt_name = os.listdir(path=ckpt_folder)[-1]
ckpt_path = os.path.join(ckpt_folder, ckpt_name)
print(Back.GREEN + "Model loaded from: <{}>".format(ckpt_path))
model = ResNet8_PL(settings=settings, num_labels=len(labels), loss_weights=None).cuda(settings.training.device)  # Load model
#'''
ckpt = torch.load(ckpt_path, lambda s, l: s)
state_dict = ckpt["state_dict"]
del state_dict["loss_fn.weight"]
model.load_state_dict(state_dict=state_dict)
#'''
# model = ResNet8_PL.load_from_checkpoint(checkpoint_path=settings.model.resnet8.pretrain_path, settings=settings, num_labels=len(labels), loss_weights=balanced_weights)
print(Back.BLUE + "LOAD PRETRAINED MODEL: {}".format(settings.model.resnet8.pretrain_path))

# Set the test dataloader
model.set_test_dataloaders(dataloaders=test_loaders)

########################
#   Setting Trainer    #
########################
trainer = pl.Trainer(
    # auto_lr_find=settings.training.lr.auto_find,
    accelerator=settings.training.accelerator,
    devices=[settings.training.device],
    # logger=logger,
    # max_epochs=settings.training.max_epochs,
    # min_epochs=settings.training.min_epochs,
    # track_grad_norm=2,
    # log_every_n_steps=-1,
    enable_model_summary=True
    # reload_dataloaders_every_epoch=False,       # set True to shuffle the dataloader before start each epoch
    # profiler=profiler,                          # set to True to see how many time was spent from the training process during the training                           # True to activate Tensorboard logger
    # weights_summary="top",                      # set to "full" to see all the weights of each layer of the network
    # benchmark=False,                            # set True if the size of the input does not change in order to speed up the training process
    # fast_dev_run=False,                         # set True to check each line of the model and training process, it is useful after some change
    # overfit_batches=1,                              # set to a number of batch on which overfit in order to understand if the training work well
    # detect_anomaly=False,
    # num_sanity_val_steps=0                      # before training, the sanity validation control performs N validation steps to check if the validation step is working correctly
)

########################
#      Test Model      #
########################
# model = ResNet8_PL.load_from_checkpoint(settings.testing.ckpt_path, num_labels=len(labels))
trainer.test(model=model, dataloaders=test_loaders, verbose=True, datamodule=None)