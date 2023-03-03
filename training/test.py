import os
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore

import torch
import pytorch_lightning as pl

from utils.dataloaders import TestingMiviaDataset, _val_collate_fn
from settings.conf_1 import settings

from models.resnet8 import ResNet8_PL
from models.mobilenetv2 import MobileNetV2_PL
from models.conformer import Conformer_PL


########################
#     Setting CUDA     #
########################
if torch.cuda.is_available():
    print(Back.GREEN + "CUDA acceleration available on devices: {}".format(settings.training.devices))
# Set CUDA device
devices = ""
for device in settings.training.devices:
    devices += str(device) + ", "
devices = devices[:-2]
os.environ["CUDA_VISIBLE_DEVICES"] = devices
pin_memory = True if settings.training.device=="cuda" else False

########################
#     Setting Seed     #
########################
pl.seed_everything(220295)

#########################
# Building Dataloaders  #
#########################
test_loaders = list()
for fold in range(settings.testing.n_folds):
    test_set = TestingMiviaDataset(fold=fold)
    test_loaders.append(torch.utils.data.DataLoader(dataset=test_set, batch_size=settings.training.batch_size, shuffle=None, num_workers=settings.training.num_workers, collate_fn=_val_collate_fn, pin_memory=pin_memory))

#########################
#    Building Model     #
#########################
assert settings.model.network in ["resnet8", "mobilenetv2"]
model = None
labels = test_set._get_labels()
if settings.model.network == "resnet8":
    model = ResNet8_PL(num_labels=len(labels)).cuda()  # Load model
    if settings.model.pretrain:
        # model.load_state_dict(torch.load(settings.model.resnet8.pretrain_path, lambda s, l: s))
        model = ResNet8_PL.load_from_checkpoint(settings.model.resnet8.pretrain_path, num_labels=len(labels))
        print(Back.BLUE + "LOAD PRETRAINED MODEL: {}".format(settings.model.resnet8.pretrain_path))
    # model.set_parameters(num_labels=len(labels), loss_weights=balanced_weights)
elif settings.model.network == "mobilenetv2":
    model = MobileNetV2_PL(num_labels=len(labels)).cuda()
    if settings.model.pretrain:
        model.load_state_dict(torch.load(settings.model.mobilenetv2.pretrain_path, lambda s, l: s))
        #model = LitModel.load_from_checkpoint(settings.model.mobilenetv2.pretrain_path, in_dim=128, out_dim=len(labels))   # (B x C x F x T) -> (128 x 1 x 64 x )
        print(Back.BLUE + "LOAD PRETRAINED MODEL: {}".format(settings.model.mobilenetv2.pretrain_path))
    model.set_parameters(num_labels=len(labels))
elif settings.model.network == "conformer":
    model = Conformer_PL(num_labels=len(labels)).cuda()

########################
#   Setting Trainer    #
########################
trainer = pl.Trainer(
    resume_from_checkpoint=None,                    # Insert a path of the ".ckpt" file to resume training from a specific checkpoint
    # auto_lr_find=settings.training.lr.auto_find,
    accelerator=settings.training.accelerator,
    devices=settings.training.devices,
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
model = ResNet8_PL.load_from_checkpoint(settings.testing.ckpt_path, num_labels=len(labels))
model.set_test_dataloaders(dataloaders=test_loaders)
trainer.test(model=model, dataloaders=test_loaders, ckpt_path=settings.testing.ckpt_path, verbose=True, datamodule=None)