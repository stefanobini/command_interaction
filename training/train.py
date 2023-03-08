import os
import shutil
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
from datetime import datetime

import torch
from pytorch_lightning import loggers
import pytorch_lightning as pl
from pytorch_lightning.profiler import SimpleProfiler, AdvancedProfiler

from utils.dataloaders import TrainingMiviaDataset, ValidationMiviaDataset, _train_collate_fn, _val_collate_fn, _MT_train_collate_fn, _MT_val_collate_fn
from settings.conf_1 import settings

from models.resnet8 import ResNet8_PL
from models.mobilenetv2 import MobileNetV2_PL
from models.conformer import Conformer_PL
from models.multitask import Multitask_SCR_SI


torch.autograd.detect_anomaly()


########################
#     Setting CUDA     #
########################
if torch.cuda.is_available():
    print(Back.GREEN + "CUDA acceleration available on {} devices, you select {}".format(torch.cuda.device_count(), settings.training.devices))
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
pl.seed_everything(5138)


########################
#   Setting Profiler   #
########################
# now = datetime.now()
# version = now.strftime("%m_%d_%Y-%H_%M_%S")
version = settings.noise.curriculum_learning.distribution
logger = loggers.TensorBoardLogger(save_dir=settings.logger.folder, name=settings.logger.name, version=version)
info_path = os.path.join(settings.logger.folder, settings.logger.name, version)
profiler = AdvancedProfiler(dirpath=info_path, filename="profiler_summary.txt")


#########################
# Building Dataloaders  #
#########################
train_set = TrainingMiviaDataset()
val_set = ValidationMiviaDataset()
commands = train_set._get_labels()
speakers = train_set._get_speakers()
# print(labels)

###### Decomment following rows if you use also reject, and change annotation file path in settings #######
weights = train_set._get_class_weights()
# Computing the label weights to balance the dataloader
dataset_weights = [((1-settings.training.reject_percentage)/len(commands)) for i in range(len(commands)-1)]
dataset_weights.append(settings.training.reject_percentage)
dataset_weights = torch.tensor(dataset_weights)
###########################################################################################################

balanced_weights = [1/len(commands) for i in range(len(commands))]
balanced_weights = torch.tensor(balanced_weights)

# dataset_weights = [((1-settings.training.reject_percentage)/len(labels)) for i in range(len(labels))]
train_sampler = torch.utils.data.sampler.WeightedRandomSampler(weights=balanced_weights, num_samples=len(train_set))
if settings.model.network == "multitask_scr_si":
    train_collate_fn = _MT_train_collate_fn
    val_collate_fn = _MT_val_collate_fn
else:
    train_collate_fn = _train_collate_fn
    val_collate_fn = _val_collate_fn
train_loader = torch.utils.data.DataLoader(dataset=train_set, batch_size=settings.training.batch_size, sampler=train_sampler, num_workers=settings.training.num_workers, collate_fn=train_collate_fn, pin_memory=pin_memory)
train_loader = torch.utils.data.DataLoader(dataset=train_set, batch_size=settings.training.batch_size, shuffle=None, num_workers=settings.training.num_workers, collate_fn=train_collate_fn, pin_memory=pin_memory)
val_loader = torch.utils.data.DataLoader(dataset=val_set, batch_size=settings.training.batch_size, shuffle=None, num_workers=settings.training.num_workers, collate_fn=val_collate_fn, pin_memory=pin_memory)


#########################
#    Building Model     #
#########################
assert settings.model.network in ["resnet8", "mobilenetv2"]
model = None
if settings.model.network == "resnet8":
    model = ResNet8_PL(num_labels=len(commands), loss_weights=balanced_weights).cuda()  # Load model
    if settings.model.pretrain:
        # model.load_state_dict(torch.load(settings.model.resnet8.pretrain_path, lambda s, l: s))
        model = ResNet8_PL.load_from_checkpoint(settings.model.resnet8.pretrain_path, num_labels=len(commands), loss_weights=balanced_weights)
        print(Back.BLUE + "LOAD PRETRAINED MODEL: {}".format(settings.model.resnet8.pretrain_path))
    # model.set_parameters(num_labels=len(labels), loss_weights=balanced_weights)
elif settings.model.network == "mobilenetv2":
    model = MobileNetV2_PL(num_labels=len(commands), loss_weights=balanced_weights).cuda()
    if settings.model.pretrain:
        model.load_state_dict(torch.load(settings.model.mobilenetv2.pretrain_path, lambda s, l: s))
        #model = LitModel.load_from_checkpoint(settings.model.mobilenetv2.pretrain_path, in_dim=128, out_dim=len(labels))   # (B x C x F x T) -> (128 x 1 x 64 x )
        print(Back.BLUE + "LOAD PRETRAINED MODEL: {}".format(settings.model.mobilenetv2.pretrain_path))
    model.set_parameters(num_labels=len(commands), loss_weights=balanced_weights)
elif settings.model.network == "conformer":
    model = Conformer_PL(num_labels=len(commands)).cuda()
elif settings.model.network == "multitask_scr_si":
    model = Multitask_SCR_SI(num_commands=len(commands), num_speakers=len(speakers), command_loss_weights=None, speaker_loss_weights=None)
model.set_train_dataloader(dataloader=train_loader)
model.set_val_dataloader(dataloader=val_loader)


########################
#   Setting Trainer    #
########################
trainer = pl.Trainer(
    resume_from_checkpoint=None,                    # Insert a path of the ".ckpt" file to resume training from a specific checkpoint
    auto_lr_find=settings.training.lr.auto_find,
    accelerator=settings.training.accelerator,
    devices=settings.training.devices,
    logger=logger,
    max_epochs=settings.training.max_epochs,
    min_epochs=settings.training.min_epochs,
    track_grad_norm=2,
    log_every_n_steps=-1,
    enable_model_summary=False,
    # reload_dataloaders_every_epoch=False,       # set True to shuffle the dataloader before start each epoch
    # profiler=profiler,                          # set to True to see how many time was spent from the training process during the training                           # True to activate Tensorboard logger
    # weights_summary="top",                      # set to "full" to see all the weights of each layer of the network
    benchmark=False,                            # set True if the size of the input does not change in order to speed up the training process
    fast_dev_run=False,                         # set True to check each line of the model and training process, it is useful after some change
    # overfit_batches=1,                              # set to a number of batch on which overfit in order to understand if the training work well
    detect_anomaly=False,
    num_sanity_val_steps=0                      # before training, the sanity validation control performs N validation steps to check if the validation step is working correctly
)

if settings.training.lr.auto_find:
    trainer.tune(model=model, train_dataloaders=model.train_loader, val_dataloaders=model.val_loader)


########################
#    Training Model    #
########################
trainer.fit(model=model, train_dataloaders=model.train_loader, val_dataloaders=model.val_loader)