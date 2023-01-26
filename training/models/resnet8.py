from math import ceil
import torch
import torchmetrics
from torch.optim.lr_scheduler import ReduceLROnPlateau
import torch.nn.functional as F
import pytorch_lightning as pl
from pytorch_lightning.callbacks import EarlyStopping, ModelCheckpoint, LearningRateMonitor

import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
from typing import Dict, List

from settings.conf_1 import settings


class ResNet8(pl.LightningModule):
    """ """

    def __init__(self, num_labels: int):
        """ """
        super().__init__()

        out_channel = settings.model.resnet8.out_channel

        self.conv0 = torch.nn.Conv2d(1, out_channel, (3, 3), padding=(1, 1), bias=False)
        self.pool = torch.nn.AvgPool2d(settings.model.resnet8.pooling_size)  # flipped -- better for 80 log-Mels

        self.n_layers = n_layers = 6
        self.convs = [torch.nn.Conv2d(out_channel, out_channel, (3, 3), padding=1, bias=False) for _ in range(n_layers)]
        for i, conv in enumerate(self.convs):
            self.add_module(f'bn{i + 1}', torch.nn.BatchNorm2d(out_channel, affine=False))
            self.add_module(f'conv{i + 1}', conv)
        self.output = torch.nn.Linear(out_channel, num_labels)
        self.softmax = torch.nn.Softmax(dim=1)

        self.num_labels = num_labels
        self.loss_fn = torch.nn.CrossEntropyLoss()
        self.batch_size = settings.training.batch_size
        self.learning_rate = settings.training.lr
        self.metric_to_track = settings.training.metric_to_track
        self.check_val_every_n_epoch = settings.training.check_val_every_n_epoch
        self.optimization_mode = settings.training.optimization_mode
        self.early_stop_patience = settings.training.early_stop.patience

        self.snrs = range(settings.noise.min_snr, settings.noise.max_snr+settings.noise.snr_step, settings.noise.snr_step)
        

    def forward(self, x:torch.FloatTensor) -> torch.FloatTensor:
        """ """
        #print(Back.BLUE + "ResNet - input shape: {}".format(x.size()))

        x = x[:, :1]  # log-Mels only
        #print(Back.BLUE + "ResNet - 'log-Mels only' shape: {}".format(x.size()))
        x = x.permute(0, 1, 3, 2).contiguous()  # Original res8 uses (time, frequency) format
        #print(Back.BLUE + "ResNet - after permutation shape: {}".format(x.size()))
        
        for i in range(self.n_layers + 1):
            y = torch.nn.functional.relu(getattr(self, f'conv{i}')(x))
            if i == 0:
                if hasattr(self, 'pool'):
                    y = self.pool(y)
                old_x = y
            if i > 0 and i % 2 == 0:
                x = y + old_x
                old_x = x
            else:
                x = y
            if i > 0:
                x = getattr(self, f'bn{i}')(x)
                
        x = x.view(x.size(0), x.size(1), -1)  # shape: (batch, feats, o3)
        x = torch.mean(x, 2)

        return self.output(x)


    def train_dataloader(self, dataloader:torch.utils.data.DataLoader) -> torch.utils.data.DataLoader:
        self.train_loader = dataloader
        self.train_it_per_epoch = len(self.train_loader)
        self.train_epoch_loss = torch.zeros(self.train_it_per_epoch).cuda()
        return self.train_loader

    def val_dataloader(self, dataloaders:List[torch.utils.data.DataLoader]) -> pl.trainer.supporters.CombinedLoader:
        self.val_loaders = pl.trainer.supporters.CombinedLoader(dataloaders)
        self.val_it_per_epoch = len(dataloaders[0])
        self.val_epoch_loss = torch.zeros(self.val_it_per_epoch).cuda()
        # used for performance computation, it contains a list of prediction and target for each SNR
        self.val_outputs = {snr:{"preds":torch.zeros(size=(self.val_it_per_epoch, self.batch_size, self.num_labels)).cuda(), "targs":torch.zeros(size=(self.val_it_per_epoch, self.batch_size)).cuda()} for snr in self.snrs}
        return self.val_loaders

    def test_dataloader(self, dataset, sampler, collate_fn, pin_memory) -> torch.utils.data.DataLoader:
        self.test_loader = torch.utils.data.DataLoader(dataset=dataset, batch_size=settings.training.batch_size, sampler=sampler, num_workers=settings.training.num_workers, collate_fn=collate_fn, pin_memory=pin_memory)
        return self.test_loader


    def get_train_loader(self) -> torch.utils.data.Dataset:
        return self.train_loader

    def get_val_loader(self) -> Dict[int, torch.utils.data.Dataset]:
        return self.val_loaders
    
    def get_test_loader(self) -> torch.utils.data.Dataset:
        return self.test_loader


    def on_train_epoch_start(self) -> None:
        self.train_step = 0


    def training_step(self, batch, batch_idx):
        """ """
        # print(Back.BLUE + "Train step, batch size: {}".format(batch[0][0].size()))
        x, y = batch
        y_pred = self.forward(x=x)
        loss = self.loss_fn(y_pred, y)

        self.log("train_loss_step", loss)
        
        # Save loss every iteration to compute the average of the overall epoch
        self.train_epoch_loss[self.train_step] = loss
        self.train_step += 1

        return loss

    
    def on_train_epoch_end(self) -> None:
        train_epoch_loss = torch.mean(self.train_epoch_loss)
        self.train_epoch_loss = torch.zeros(self.train_it_per_epoch).cuda()

        self.log("train_loss_epoch", train_epoch_loss)    # save train loss on tensorboard logger

        self.train_loader.dataset._shuffle_noise_dataset()  # shuffle noise sample among epochs
        self.train_loader.dataset.increase_epoch(step=1)


    def on_validation_epoch_start(self) -> None:
        self.val_step = 0

    
    def validation_step(self, batches, batch_idx):
        """ """
        # print(self.val_step)
        # print(Back.YELLOW + "VALIDATION STEP\ndataloader: {}\nbatch size: x:{}, y:{}\n".format(batches.keys(), batches[0][0].size(), batches[0][1].size()))
        losses = torch.zeros(len(self.snrs)).cuda()
        for i, snr in zip(range(len(self.snrs)), batches):    # for batch of each snr
            x, y = batches[snr]
            y_pred = self.forward(x=x)
            loss = self.loss_fn(y_pred, y)
            losses[i] = loss
            y_pred = self.softmax(y_pred)
            '''
            if y_pred.size(0) != self.batch_size:
                pad = (0, 0, 0, self.batch_size-y_pred.size(0))
                y_pred = F.pad(input=y_pred, pad=pad, mode="constant", value=0)
                print(y.size(), pad)
                y = F.pad(input=y, pad=pad, mode="constant", value=0)
            '''
            # print("preds: {}\ny_pred: {}\ntargs: {}\ny: {}\n".format(self.val_outputs[snr]["preds"][self.val_step].size(), y_pred.size(), self.val_outputs[snr]["targs"][self.val_step].size(), y.size()))
            if y_pred.size(0) == self.batch_size:
                self.val_outputs[snr]["preds"][self.val_step] = y_pred
                self.val_outputs[snr]["targs"][self.val_step] = y
        
        loss = torch.mean(losses)
        self.log("val_loss_step", loss)

        # Save loss every iteration to compute the average of the overall epoch
        self.val_epoch_loss[self.val_step] = loss
        self.val_step += 1

        return loss


    def validation_epoch_end(self, val_step_outputs) -> None:
        epoch_loss = torch.mean(self.val_epoch_loss)
        self.log("val_loss_epoch", epoch_loss)
        self.val_epoch_loss = torch.zeros(self.val_it_per_epoch).cuda() # reset the val_epoch_loss for the next epoch

        accuracy_mean = list()
        balanced_accuracy_mean = list()
        reject_accuracy_mean = list()

        # Metrics computation
        for snr in self.snrs:
            # print("VALIDATION METRICS\npredictions: {}\ntargets: {}\n".format(self.val_outputs[snr]["preds"].size(), self.val_outputs[snr]["targs"].size()))
            predictions = torch.reshape(input=self.val_outputs[snr]["preds"], shape=(-1, self.num_labels))
            targets = torch.reshape(input=self.val_outputs[snr]["targs"], shape=(-1,))
            # print(self.val_outputs[snr]["preds"].size(), predictions.size(), self.val_outputs[snr]["targs"].size(), targets.size(), numpy.unique(targets.cpu()))
            accuracy = torchmetrics.functional.classification.accuracy(preds=predictions, target=targets, task="multiclass", num_classes=self.num_labels, average="micro")
            balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=predictions, target=targets, task="multiclass", num_classes=self.num_labels, average="weighted")
            
            pred_reject_y = torch.zeros(size=(self.val_it_per_epoch, self.batch_size))
            pred_reject_y = self.val_outputs[snr]["preds"][:, :, self.num_labels-1]                 # '1' for reject, '0' for command
            pred_reject_y = torch.reshape(input=pred_reject_y, shape=(-1,))
            targ_reject_y = self.val_outputs[snr]["targs"] == (self.num_labels-1)             # '1' for reject, '0' for command
            targ_reject_y = torch.reshape(input=targ_reject_y, shape=(-1,))
            reject_accuracy = torchmetrics.functional.classification.binary_accuracy(preds=pred_reject_y, target=targ_reject_y)
            
            accuracy_mean.append(accuracy)
            balanced_accuracy_mean.append(balanced_accuracy)
            reject_accuracy_mean.append(reject_accuracy)
            
            self.log("accuracy_{}_dB".format(snr), accuracy)
            self.log("balanced_accuracy_{}_dB".format(snr), balanced_accuracy)
            self.log("reject_accuracy_{}_dB".format(snr), reject_accuracy)
        
        self.log("accuracy", sum(accuracy_mean)/len(accuracy_mean))
        self.log("balanced_accuracy", sum(balanced_accuracy_mean)/len(balanced_accuracy_mean))
        self.log("reject_accuracy", sum(reject_accuracy_mean)/len(reject_accuracy_mean))
        
        self.val_outputs = {snr:{"preds":torch.zeros(size=(self.val_it_per_epoch, self.batch_size, self.num_labels)).cuda(), "targs":torch.zeros(size=(self.val_it_per_epoch, self.batch_size)).cuda()} for snr in self.snrs}
    

    def test_step(self, batch, batch_idx):
        """ """
        
        x, y = batch
        y_pred = self.forward(x=x)
        loss = self.loss_fn(y_pred, y)
        self.log("test_loss", loss)
    

    def configure_callbacks(self):
        early_stop = EarlyStopping(monitor=self.metric_to_track, patience=self.early_stop_patience, verbose=True, mode=self.optimization_mode)
        lr_monitor = LearningRateMonitor(logging_interval="epoch")
        checkpoint = ModelCheckpoint(monitor=self.metric_to_track)
        return [early_stop, lr_monitor, checkpoint]


    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=self.learning_rate)
        scheduler = ReduceLROnPlateau(optimizer=optimizer, mode=self.optimization_mode, patience=settings.training.scheduler.patience, verbose=True)
        optimizers_schedulers = {
            "optimizer": optimizer,
            "lr_scheduler": {
                "scheduler": scheduler,
                "monitor": self.metric_to_track,
                "frequency": self.check_val_every_n_epoch
            }
        }
        return optimizers_schedulers