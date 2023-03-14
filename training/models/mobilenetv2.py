from typing import Dict

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.optim.lr_scheduler import ReduceLROnPlateau
import torchmetrics
from torchvision.models import MobileNetV2, mobilenet_v2
import pytorch_lightning as pl
from pytorch_lightning.callbacks import EarlyStopping, ModelCheckpoint, LearningRateMonitor

from settings.SCR_conf import settings


class MobileNetV2_PL(pl.LightningModule):
    def __init__(self, num_labels: int, loss_weights:torch.FloatTensor):
        super().__init__()
        self.downsample = nn.Sequential(nn.Conv2d(1, 3, 3, padding=(1, 3)),
                                        nn.BatchNorm2d(3),
                                        nn.ReLU(),
                                        nn.MaxPool2d((1, 2)))
        self.model = mobilenet_v2(pretrained=False)
        model = MobileNetV2(num_classes=30)
        self.model.classifier = model.classifier
    

    def set_parameters(self, num_labels: int, loss_weights:torch.FloatTensor):
        model = MobileNetV2(num_classes=num_labels)
        self.model.classifier = model.classifier
        self.softmax = torch.nn.Softmax(dim=1)

        self.num_labels = num_labels
        self.loss_fn = torch.nn.CrossEntropyLoss(weight=loss_weights)
        self.batch_size = settings.training.batch_size
        self.learning_rate = settings.training.lr.value
        self.metric_to_track = settings.training.checkpoint.metric_to_track
        self.check_val_every_n_epoch = settings.training.check_val_every_n_epoch
        self.optimization_mode = settings.training.optimization_mode
        self.early_stop_patience = settings.training.early_stop.patience

        self.snrs = list(range(settings.noise.min_snr, settings.noise.max_snr+settings.noise.snr_step, settings.noise.snr_step))
        

    def forward(self, x):
        x = x[:, :1]  # log-Mels only
        x = self.downsample(x)
        return self.model(x)

    def train_dataloader(self, dataloader:torch.utils.data.DataLoader) -> torch.utils.data.DataLoader:
        self.train_loader = dataloader
        self.train_preds, self.train_targs = list(), list()
        return self.train_loader

    def val_dataloader(self, dataloader:torch.utils.data.DataLoader) -> torch.utils.data.DataLoader:
        self.val_loader = dataloader
        self.val_preds, self.val_targs, self.val_snrs = list(), list(), list()
        return self.val_loader

    def test_dataloader(self, dataloader:torch.utils.data.DataLoader) -> torch.utils.data.DataLoader:
        self.test_loader = dataloader
        return self.test_loader


    def get_train_loader(self) -> torch.utils.data.Dataset:
        return self.train_loader

    def get_val_loader(self) -> torch.utils.data.Dataset:
        return self.val_loader
    
    def get_test_loader(self) -> torch.utils.data.Dataset:
        return self.test_loader


    def training_step(self, batch, batch_idx) -> torch.FloatTensor:
        """ """
        # print(Back.BLUE + "Train step, batch size: {}".format(batch[0][0].size()))
        x, y = batch
        y_pred = self.forward(x=x)
        loss = self.loss_fn(input=y_pred, target=y)
        
        self.train_preds += y_pred
        self.train_targs += y

        #print("Trainig - preds: {}, targs: {}".format(len(self.train_preds), len(self.train_targs)))

        return loss

    
    def on_train_epoch_end(self) -> None:
        """ """
        preds = torch.stack(self.train_preds).cuda()
        targs = torch.tensor(self.train_targs).cuda()

        loss = self.loss_fn(input=preds, target=targs)
        self.log("train_loss", loss, on_epoch=True, prog_bar=True, logger=True)    # save train loss on tensorboard logger

        #print("Training - preds: {}, targs: {}, loss: {}".format(preds.size(), targs.size(), loss))

        self.train_loader.dataset._shuffle_noise_dataset()  # shuffle noise sample among epochs
        self.train_loader.dataset.increase_epoch(step=1)

        '''
        with open("train_preds.txt", "wb") as fout:
            pickle.dump(self.train_preds, fout)
        with open("train_targs.txt", "wb") as fout:
            pickle.dump(self.train_targs, fout)
        '''

        self.train_preds, self.train_targs = list(), list() # Clean the lists of epoch training results

    
    def validation_step(self, batch, batch_idx) -> Dict[str, torch.Tensor]:
        """Callback function activated in each validation step. It accumulate validation epoch results in the validation result lists (self.val_preds, self.val_targs, self.val_snrs)
        
        Parameters
        ----------
        batch: torch.FloatTensor
            Batch tensor of samples.
        """
        # print(Back.YELLOW + "VALIDATION STEP\ndataloader: {}\nbatch size: x:{}, y:{}\n".format(batches.keys(), batches[0][0].size(), batches[0][1].size()))
        x, y, snr = batch
        pred = self.forward(x=x)
        #pred = self.softmax(pred)

        self.val_preds += pred
        self.val_targs += y
        self.val_snrs += snr
        #print("Validation - preds: {}, targs: {}, snr: {}".format(len(self.val_preds), len(self.val_targs), len(self.val_snrs)))


    def validation_epoch_end(self, val_outputs) -> None:  
        preds = torch.stack(self.val_preds).cuda()
        # preds = torch.max(input=preds, dim=1).indices    # take only the most likely class
        targs = torch.tensor(self.val_targs).cuda()
        #targs_one_hot = F.one_hot(input=targs, num_classes=self.num_labels).type(torch.FloatTensor).cuda()  # resize targets in order to compute the metrics on the probability vectors
        snrs = torch.tensor(self.val_snrs)

        # Compute loss and metrics
        loss = self.loss_fn(input=preds, target=targs)
        preds = torch.max(input=preds, dim=1).indices
        accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targs, task="multiclass", num_classes=self.num_labels, average="micro")
        balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targs, task="multiclass", num_classes=self.num_labels, average="weighted")
        #accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targs_one_hot, task="multiclass", num_classes=self.num_labels, average="micro")
        #balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targs_one_hot, task="multiclass", num_classes=self.num_labels, average="weighted")
        #pred_reject_y = torch.max(input=preds, dim=1).indices
        #targ_reject_y = torch.max(input=targs_one_hot, dim=1).indices
        #pred_reject_y = pred_reject_y == (self.num_labels-1)             # '1' for reject, '0' for command
        #targ_reject_y = targ_reject_y == (self.num_labels-1)            # '1' for reject, '0' for command
        pred_reject_y = preds == (self.num_labels-1)             # '1' for reject, '0' for command
        targ_reject_y = targs == (self.num_labels-1)            # '1' for reject, '0' for command
        reject_accuracy = torchmetrics.functional.classification.binary_accuracy(preds=pred_reject_y, target=targ_reject_y)
        
        #print("Validation - preds: {}, targs: {}, snr: {}, loss: {}, accuracy: {}, balanced accuracy: {}, reject accuracy: {}".format(preds.size(), targs.size(), snrs.size(), loss, accuracy, balanced_accuracy, reject_accuracy))

        # Save for TensorBoard
        self.log("val_loss", loss, on_epoch=True, prog_bar=True, logger=True)
        self.log("accuracy", accuracy, on_epoch=True, prog_bar=True, logger=True)
        self.log("balanced_accuracy", balanced_accuracy, on_epoch=True, prog_bar=True, logger=True)
        self.log("reject_accuracy", reject_accuracy, on_epoch=True, prog_bar=True, logger=True)

        # Construct and fill dictionary that contains the predictions and target devided by SNR
        outputs = dict()
        for snr in self.snrs:
            outputs[snr] = {"preds": list(), "targs": list()}
        for idx in range(len(snrs)):
            outputs[snrs[idx].item()]["preds"].append(preds[idx])
            #outputs[snrs[idx].item()]["targs"].append(targs_one_hot[idx])
            outputs[snrs[idx].item()]["targs"].append(targs[idx])
        
        # Convert lists in tensors
        for snr in self.snrs:    
            outputs[snr]["preds"] = torch.stack(outputs[snr]["preds"])
            outputs[snr]["targs"] = torch.stack(outputs[snr]["targs"])
        
        # Compute metrics for each SNR
        for snr in self.snrs:
            snr_accuracy = torchmetrics.functional.classification.accuracy(preds=outputs[snr]["preds"], target=outputs[snr]["targs"], task="multiclass", num_classes=self.num_labels, average="micro")
            snr_balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=outputs[snr]["preds"], target=outputs[snr]["targs"], task="multiclass", num_classes=self.num_labels, average="weighted")
            #pred_reject_y = torch.max(input=outputs[snr]["preds"], dim=1).indices
            #targ_reject_y = torch.max(input=outputs[snr]["targs"], dim=1).indices
            #pred_reject_y = pred_reject_y == (self.num_labels-1)    # '1' for reject, '0' for command
            #targ_reject_y = targ_reject_y == (self.num_labels-1)    # '1' for reject, '0' for command
            pred_reject_y = outputs[snr]["preds"] == (self.num_labels-1)    # '1' for reject, '0' for command
            targ_reject_y = outputs[snr]["targs"] == (self.num_labels-1)    # '1' for reject, '0' for command
            snr_reject_accuracy = torchmetrics.functional.classification.binary_accuracy(preds=pred_reject_y, target=targ_reject_y)
        
            self.log("accuracy_{}_dB".format(snr), snr_accuracy, on_epoch=True, logger=True)
            self.log("balanced_accuracy_{}_dB".format(snr), snr_balanced_accuracy, on_epoch=True, logger=True)
            self.log("reject_accuracy_{}_dB".format(snr), snr_reject_accuracy, on_epoch=True, logger=True)

        '''
        with open("val_preds.txt", "wb") as fout:
            pickle.dump(self.val_preds, fout)
        with open("val_targs.txt", "wb") as fout:
            pickle.dump(self.val_targs, fout)
        with open("val_rjts.txt", "wb") as fout:
            pickle.dump(targ_reject_y, fout)
        '''

        self.val_preds, self.val_targs, self.val_snrs = list(), list(), list()      # Clean the lists of epoch validation results
    

    def test_step(self, batch, batch_idx):
        """ """
        
        x, y = batch
        y_pred = self.forward(x=x)
        loss = self.loss_fn(y_pred, y)
        self.log("test_loss", loss)
    

    def configure_callbacks(self):
        early_stop = EarlyStopping(monitor=self.metric_to_track, patience=self.early_stop_patience, verbose=True, mode=self.optimization_mode, check_finite=True, check_on_train_epoch_end=False)
        lr_monitor = LearningRateMonitor(logging_interval="epoch")
        checkpoint = ModelCheckpoint(monitor=self.metric_to_track)
        return [early_stop, lr_monitor, checkpoint]


    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=self.learning_rate)
        scheduler = ReduceLROnPlateau(optimizer=optimizer, mode=self.optimization_mode, patience=settings.training.scheduler.patience, eps=1e-4, verbose=True)
        optimizers_schedulers = {
            "optimizer": optimizer,
            "lr_scheduler": {
                "scheduler": scheduler,
                "monitor": self.metric_to_track,
                "frequency": self.check_val_every_n_epoch
            }
        }
        return optimizers_schedulers