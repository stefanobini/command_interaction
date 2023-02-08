from math import ceil
from typing import Dict
import pickle
import torch
import torchmetrics
from torch.optim.lr_scheduler import ReduceLROnPlateau
from torch.nn.functional import cross_entropy
import pytorch_lightning as pl
from pytorch_lightning.callbacks import EarlyStopping, ModelCheckpoint, LearningRateMonitor
import torchvision
import torch_optimizer

import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
from typing import Dict, List

from settings.conf_1 import settings


class ResNet8_PL(pl.LightningModule):
    """ """

    def __init__(self, num_labels:int, loss_weights:torch.FloatTensor):
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
        self.output = torch.nn.Linear(out_channel, 30)
        

    def set_parameters(self, num_labels: int, loss_weights:torch.FloatTensor):
        out_channel = settings.model.resnet8.out_channel
        self.output = torch.nn.Linear(out_channel, num_labels)
        # self.softmax = torch.nn.Softmax(dim=1)
        self.loss_fn = torch.nn.CrossEntropyLoss(weight=loss_weights)

        self.num_labels = num_labels
        self.batch_size = settings.training.batch_size
        self.learning_rate = settings.training.lr.value
        self.metric_to_track = settings.training.metric_to_track
        self.check_val_every_n_epoch = settings.training.check_val_every_n_epoch
        self.optimization_mode = settings.training.optimizer.mode
        self.early_stop_patience = settings.training.early_stop.patience

        self.snrs = list(range(settings.noise.min_snr, settings.noise.max_snr+settings.noise.snr_step, settings.noise.snr_step))
        

    def forward(self, x:torch.FloatTensor) -> torch.FloatTensor:
        """Input size: (Batch, Channel, Frequency, Time)"""
        #print(Back.BLUE + "ResNet - input shape: {}".format(x.size()))

        if settings.input.normalize:
            x = torch.nn.functional.normalize(input=x)

        x = x[:, :1]  # log-Mels only
        #print(Back.BLUE + "ResNet - 'log-Mels only' shape: {}".format(x.size()))
        x = x.permute(0, 1, 3, 2).contiguous()  # Original res8 uses (time, frequency) format -> from (B, C, F, T) to (B, C, T, F)
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
        return self.train_loader

    def val_dataloader(self, dataloader:torch.utils.data.DataLoader) -> torch.utils.data.DataLoader:
        self.val_loader = dataloader
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
    

    @torch.no_grad()
    def on_train_epoch_start(self) -> None:
        """ """
        self.train_loader.dataset._shuffle_noise_dataset()              # shuffle noise sample among epochs
        self.train_loader.dataset.set_epoch(epoch=self.current_epoch)

    def training_step(self, batch, batch_idx) -> torch.FloatTensor:
        """ """
        # print(Back.BLUE + "Train step, batch size: {}".format(batch[0][0].size()))
        x, y = batch
        logits = self.forward(x=x)
        #print("preds: ", pred[:10])
        #print("targs: ", y[:10])
        #loss = cross_entropy(input=pred, target=y)
        loss = self.loss_fn(input=logits, target=y)

        # self.log("train_loss", loss, on_step=True, on_epoch=True, prog_bar=True, logger=True)    # save train loss on tensorboard logger
        #print("Trainig - preds: {}, targs: {}".format(logits.size(), y.size()))

        train_step_output = {"loss": loss, "logits": logits, "targets": y}
        return train_step_output
    
    
    def training_epoch_end(self, train_step_outputs):
        """ """
        len_val = len(self.train_loader)              # compute size of validation set
        # Build the epoch logits, targets and snrs tensors from the values of each iteration. Start from the first batch iteration then concatenate the followings
        logits = train_step_outputs[0]["logits"]
        targets = train_step_outputs[0]["targets"]
        for i in range(1, len_val):
            logits = torch.concat(tensors=[logits, train_step_outputs[i]["logits"]])
            targets = torch.concat(tensors=[targets, train_step_outputs[i]["targets"]])
        
        loss = self.loss_fn(input=logits, target=targets)
        preds = torch.max(input=logits, dim=1).indices
        accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="micro")
        balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="weighted")

        self.log("train_loss", loss, on_epoch=True, prog_bar=True, logger=True)    # save train loss on tensorboard logger
        self.log("train_accuracy", accuracy, on_epoch=True, prog_bar=False, logger=True)
        self.log("train_balanced_accuracy", balanced_accuracy, on_epoch=True, prog_bar=False, logger=True)

    
    def validation_step(self, batch, batch_idx) -> Dict[str, torch.Tensor]:
        """Callback function activated in each validation step
        
        Parameters
        ----------
        batch: torch.FloatTensor
            Batch tensor of samples.
        """
        # print(Back.YELLOW + "VALIDATION STEP\ndataloader: {}\nbatch size: x:{}, y:{}\n".format(batches.keys(), batches[0][0].size(), batches[0][1].size()))
        x, y, snr = batch
        logits = self.forward(x=x)
        # pred = self.softmax(logits)
        #print("preds: ", pred[:10])
        #print("targs: ", y[:10])

        #print("Validation - preds: {}, targs: {}, snr: {}".format(logits.size(), y.size(), snr.size()))
        validation_step_output = {"logits": logits, "targets": y, "snrs": snr}
        return validation_step_output


    def validation_epoch_end(self, val_step_outputs) -> None:
        len_val = len(self.val_loader)              # compute size of validation set
        # Build the epoch logits, targets and snrs tensors from the values of each iteration
        logits = val_step_outputs[0]["logits"]
        targets = val_step_outputs[0]["targets"]
        snrs = val_step_outputs[0]["snrs"]
        for i in range(1, len_val):
            logits = torch.concat(tensors=[logits, val_step_outputs[i]["logits"]])
            targets = torch.concat(tensors=[targets, val_step_outputs[i]["targets"]])
            snrs = torch.concat(tensors=[snrs, val_step_outputs[i]["snrs"]])

        # Compute mean metrics
        loss = self.loss_fn(input=logits, target=targets)
        preds = torch.max(input=logits, dim=1).indices
        accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="micro")
        balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="weighted")
        '''
        pred_reject_y = preds == (self.num_labels-1)             # '1' for reject, '0' for command
        targ_reject_y = targs == (self.num_labels-1)            # '1' for reject, '0' for command
        reject_accuracy = torchmetrics.functional.classification.binary_accuracy(preds=pred_reject_y, target=targ_reject_y)
        '''

        #print("Validation - preds: {}, targs: {}, snr: {}, loss: {}, accuracy: {}, balanced accuracy: {}, reject accuracy: {}".format(preds.size(), targs.size(), snrs.size(), loss, accuracy, balanced_accuracy, reject_accuracy))

        # Save for TensorBoard
        self.log("val_loss", loss, on_epoch=True, prog_bar=True, logger=True)
        self.log("accuracy", accuracy, on_epoch=True, prog_bar=True, logger=True)
        self.log("balanced_accuracy", balanced_accuracy, on_epoch=True, prog_bar=False, logger=True)
        '''
        self.log("reject_accuracy", reject_accuracy, on_epoch=True, prog_bar=True, logger=True)
        '''

        ## Compute metrics for each SNR
        # Construct and fill dictionary that contains the predictions and target devided by SNR
        outputs = dict()
        for snr in self.snrs:
            outputs[snr] = {"preds": list(), "targs": list()}
        for idx in range(len_val):
            outputs[snrs[idx].item()]["preds"].append(preds[idx])
            outputs[snrs[idx].item()]["targs"].append(targets[idx])
        
        # Convert lists in tensors
        for snr in self.snrs:    
            outputs[snr]["preds"] = torch.stack(outputs[snr]["preds"])
            outputs[snr]["targs"] = torch.stack(outputs[snr]["targs"])
        
        # Compute metrics for each SNR
        for snr in self.snrs:
            snr_accuracy = torchmetrics.functional.classification.accuracy(preds=outputs[snr]["preds"], target=outputs[snr]["targs"], task="multiclass", num_classes=self.num_labels, average="micro")
            snr_balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=outputs[snr]["preds"], target=outputs[snr]["targs"], task="multiclass", num_classes=self.num_labels, average="weighted")
            '''
            pred_reject_y = outputs[snr]["preds"] == (self.num_labels-1)    # '1' for reject, '0' for command
            targ_reject_y = outputs[snr]["targs"] == (self.num_labels-1)    # '1' for reject, '0' for command
            snr_reject_accuracy = torchmetrics.functional.classification.binary_accuracy(preds=pred_reject_y, target=targ_reject_y)
            '''
        
            self.log("accuracy_{}_dB".format(snr), snr_accuracy, on_epoch=True, logger=True)
            self.log("balanced_accuracy_{}_dB".format(snr), snr_balanced_accuracy, on_epoch=True, logger=True)
            '''
            self.log("reject_accuracy_{}_dB".format(snr), snr_reject_accuracy, on_epoch=True, logger=True)
            '''

        '''
        with open("val_preds.txt", "wb") as fout:
            pickle.dump(preds, fout)
        with open("val_targs.txt", "wb") as fout:
            pickle.dump(targets, fout)
        with open("val_rjts.txt", "wb") as fout:
            pickle.dump(targs_rejects, fout)
        '''
    

    def test_step(self, batch, batch_idx):
        """ """
        
        x, y = batch
        y_pred = self.forward(x=x)
        #loss = cross_entropy(y_pred, y)
        loss = self.loss_fn(input=y_pred, target=y)
        self.log("test_loss", loss)
    

    def configure_callbacks(self):
        early_stop = EarlyStopping(monitor=self.metric_to_track, patience=self.early_stop_patience, verbose=True, mode=self.optimization_mode, check_finite=True, check_on_train_epoch_end=False)
        lr_monitor = LearningRateMonitor(logging_interval="epoch")
        checkpoint = ModelCheckpoint(monitor=self.metric_to_track)
        return [early_stop, lr_monitor, checkpoint]
        # return [lr_monitor, checkpoint]

 
    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=self.learning_rate, betas=settings.training.optimizer.betas, eps=settings.training.optimizer.eps, weight_decay=settings.training.optimizer.weight_decay, amsgrad=settings.training.optimizer.amsgrad)
        #optimizer = torch.optim.AdamW(self.parameters(), lr=self.learning_rate, betas=settings.training.optimizer.betas, eps=settings.training.optimizer.eps, weight_decay=settings.training.optimizer.weight_decay, amsgrad=settings.training.optimizer.amsgrad)
        #optimizer = torch_optimizer.NovoGrad(params=self.parameters(), lr=self.learning_rate, betas=settings.training.optimizer.novograd.betas, eps=settings.training.optimizer.eps, weight_decay=settings.training.optimizer.weight_decay, grad_averaging=settings.training.optimizer.grad_averaging, amsgrad=settings.training.optimizer.amsgrad)
        scheduler = ReduceLROnPlateau(optimizer=optimizer, mode=self.optimization_mode, patience=settings.training.reduce_lr_on_plateau.patience, eps=1e-4, verbose=True)
        optimizers_schedulers = {
            "optimizer": optimizer,
            "lr_scheduler": {
                "scheduler": scheduler,
                "monitor": self.metric_to_track,
                "frequency": self.check_val_every_n_epoch
            }
        }
        return optimizers_schedulers