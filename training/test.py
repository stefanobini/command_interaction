"""
python3 test.py --configuration SCR_conf 2>&1 | tee t0_log.txt
python3 test.py --configuration FELICE_conf 2>&1 | tee t0_log.txt
"""

import os
import sys
import argparse
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
import numpy as np

import torch
import pytorch_lightning as pl

from utils.dataloaders import TestingMiviaDataset, Testing_MSI, _SCR_val_collate_fn, _SI_val_collate_fn, _MT_val_collate_fn, _MSI_val_collate_fn
#from .settings.SCR_conf import settings

from models.resnet8 import ResNet8_PL
from models.mobilenetv2 import MobileNetV2_PL
from models.conformer import Conformer_PL
from models.hardsharing import HardSharing_PL
# from models.hardsharing_mobilenetv2 import HardSharing_PL
from models.softsharing import SoftSharing_PL
# from models.softsharing_mobilenetv2 import SoftSharing_PL
from models.MSI_hardsharing import HardSharing_PL as HardSharing_MSI


# Reduce the internal precision of the matrix multiplications (the type of the variable doesn't change)
torch.set_float32_matmul_precision(precision="medium")

########################
# Acquiring parameters #
########################
parser = argparse.ArgumentParser()
parser.add_argument("--configuration", type=str, dest="configuration", required=True, help="Configuration file (e.g., 'conf_1')")
args = parser.parse_args()
args.configuration = "settings.{}".format(args.configuration)
settings = getattr(__import__(args.configuration, fromlist=["settings"]), "settings")
print(Back.CYAN + "Loaded <{}> as configuration file.".format(settings.name))


########################
#     Setting CUDA     #
########################
if torch.cuda.is_available():
    available_devices = [i for i in range(torch.cuda.device_count())]
    # os.environ["CUDA_VISIBLE_DEVICES"] = str(available_devices)
    print(Back.GREEN + "CUDA acceleration available on <{}> devices: <{}>".format(torch.cuda.device_count(), available_devices))
    # print(Back.GREEN + "The current device is <{}>, you select device <{}>".format(torch.cuda.current_device(), settings.training.device))
pin_memory = True if settings.training.device=="cuda" else False

########################
#     Setting Seed     #
########################
pl.seed_everything(220295)

#########################
# Building Dataloaders  #
#########################
test_set, main_labels, test_collate_fn = None, list(), None
if settings.tasks == ["command"]:
    test_set = TestingMiviaDataset(settings=settings)
    labels = [[i for i in range(11)]] # test_set._get_labels()
    main_labels = labels[0]
    task_n_labels = len(main_labels)
    test_collate_fn = _SCR_val_collate_fn
elif settings.tasks == ["speaker"]:
    test_set = TestingMiviaDataset(settings=settings)
    labels = test_set._get_labels()
    main_labels = labels[0]
    task_n_labels = len(main_labels)
    test_collate_fn = _SI_val_collate_fn
elif settings.tasks == ["command", "speaker"]:
    test_set = TestingMiviaDataset(settings=settings)
    labels = test_set._get_labels()
    main_labels = labels[1]
    task_n_labels = list((len(labels[1]), len(labels[0])))
    test_collate_fn = _MT_val_collate_fn
elif settings.tasks == ["intent", "explicit", "implicit"]:
    test_set = Testing_MSI(settings=settings)
    labels = test_set._get_labels()
    main_labels = labels[0]
    task_n_labels = list((len(labels[0]), len(labels[1]), len(labels[2])))
    test_collate_fn = _MSI_val_collate_fn
else:
    sys.exit("The <{}> task is not allowed.".format(settings.tasks))
balanced_weights = [1/len(main_labels) for i in range(len(main_labels))]
balanced_weights = torch.tensor(balanced_weights)

test_loaders = list()
for fold in range(settings.testing.n_folds):
    test_set = Testing_MSI(settings=settings, fold=fold) if settings.tasks == ["intent", "explicit", "implicit"] else TestingMiviaDataset(settings=settings, fold=fold)
    test_loaders.append(torch.utils.data.DataLoader(dataset=test_set, batch_size=settings.training.batch_size, shuffle=None, num_workers=settings.training.num_workers, collate_fn=test_collate_fn, pin_memory=pin_memory))

#########################
#    Building Model     #
#########################
assert settings.model.network in ["resnet8", "mobilenetv2", "conformer", "HS", "SS", "HS_msi"]
model = None
if settings.model.network == "resnet8":
    model = ResNet8_PL(settings=settings, num_labels=task_n_labels, loss_weights=balanced_weights).cuda(settings.training.device)  # Load model
elif settings.model.network == "mobilenetv2":
    model = MobileNetV2_PL(settings=settings, num_labels=task_n_labels, loss_weights=balanced_weights).cuda(settings.training.device)
elif settings.model.network == "conformer":
    model = Conformer_PL(settings=settings, num_labels=task_n_labels, loss_weights=balanced_weights).cuda(settings.training.device)  # Load model
elif settings.model.network == "HS":
    model = HardSharing_PL(settings=settings, task_n_labels=task_n_labels, task_loss_weights=np.array(object=(None, None))).cuda(settings.training.device)
elif settings.model.network == "SS":
    model = SoftSharing_PL(settings=settings, task_n_labels=task_n_labels, task_loss_weights=np.array(object=(None, None))).cuda(settings.training.device)
elif settings.model.network == "HS_msi":
    model = HardSharing_MSI(settings=settings, task_n_labels=task_n_labels, task_loss_weights=np.array(object=(None, None, None))).cuda(settings.training.device)

########################
#   Setting Trainer    #
########################
trainer = pl.Trainer(
    # resume_from_checkpoint=None,                    # Insert a path of the ".ckpt" file to resume training from a specific checkpoint
    # auto_lr_find=settings.training.lr.auto_find,
    accelerator=settings.training.accelerator,
    devices=[settings.training.device],
    # logger=logger,
    # max_epochs=settings.training.max_epochs,
    # min_epochs=settings.training.min_epochs,
    # track_grad_norm=2,
    # log_every_n_steps=-1,
    enable_model_summary=False
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
model = model.load_from_checkpoint(settings.testing.ckpt_path, settings=settings, num_labels=task_n_labels, loss_weights=balanced_weights, map_location={"cuda:0":"cuda:0"})
model.set_test_dataloaders(dataloaders=test_loaders)
trainer.test(model=model, dataloaders=test_loaders, ckpt_path=settings.testing.ckpt_path, verbose=True, datamodule=None)