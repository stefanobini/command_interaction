import os
import shutil
from typing import Dict, List
from dotmap import DotMap

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

    def __init__(self, settings:DotMap, num_labels:int, loss_weights:torch.Tensor=None):
        super().__init__()
        self.downsample = nn.Sequential(nn.Conv2d(1, 3, 3, padding=(1, 3)),
                                        nn.BatchNorm2d(3),
                                        nn.ReLU(),
                                        nn.MaxPool2d((1, 2)))
        self.model = mobilenet_v2(pretrained=False)
        model = MobileNetV2(num_classes=num_labels)
        self.model.classifier = model.classifier

        self.settings = settings
        self.task = settings.task.lower()
        self.loss_weights = loss_weights
        self.loss_fn = torch.nn.CrossEntropyLoss(weight=self.loss_weights)
        self.num_labels = num_labels
        self.batch_size = settings.training.batch_size
        self.learning_rate = settings.training.lr.value
        self.snrs = list(range(settings.noise.min_snr, settings.noise.max_snr+settings.noise.snr_step, settings.noise.snr_step))

        self.save_hyperparameters()
    

    def forward(self, x):
        x = x[:, :1]  # log-Mels only
        x = self.downsample(x)
        return self.model(x)

    def set_train_dataloader(self, dataloader:torch.utils.data.DataLoader) -> torch.utils.data.DataLoader:
        """Set the training loader.
        
        Parameters
        ----------
        dataloader: torch.utils.data.DataLoader
            Training dataloader
        
        Returns
        -------
        torch.utils.data.DataLoader
            Training dataloader
        """
        self.train_loader = dataloader
        return self.train_loader

    def set_val_dataloader(self, dataloader:torch.utils.data.DataLoader) -> torch.utils.data.DataLoader:
        """Set the validation loader.
        
        Parameters
        ----------
        dataloader: torch.utils.data.DataLoader
            Validation dataloader
        
        Returns
        -------
        torch.utils.data.DataLoader
            Validation dataloader
        """
        self.val_loader = dataloader
        return self.val_loader

    def set_test_dataloaders(self, dataloaders:List[torch.utils.data.DataLoader]) -> torch.utils.data.DataLoader:
        """Set the test loader.
        
        Parameters
        ----------
        dataloader: torch.utils.data.DataLoader
            Test dataloader
        
        Returns
        -------
        torch.utils.data.DataLoader
            Test dataloader
        """
        self.test_loaders = dataloaders
        return self.test_loaders        


    def get_train_loader(self) -> torch.utils.data.Dataset:
        """Get the training loader.
        
        Returns
        -------
        torch.utils.data.DataLoader
            Training dataloader"""
        return self.train_loader

    def get_val_loader(self) -> torch.utils.data.Dataset:
        """Get the validation loader.
        
        Returns
        -------
        torch.utils.data.DataLoader
            Validation dataloader"""
        return self.val_loader
    
    def get_test_loader(self) -> torch.utils.data.Dataset:
        """Get the test loader.
        
        Returns
        -------
        torch.utils.data.DataLoader
            Test dataloader"""
        return self.test_loaders


    def on_train_start(self) -> None:
        src_conf_file = os.path.join("settings", self.settings.name.split('/')[-1])
        dst_conf_file = os.path.join(self.trainer.logger.log_dir, self.settings.name.split('/')[-1])
        shutil.copyfile(src=src_conf_file, dst=dst_conf_file)

    def on_train_epoch_start(self) -> None:
        """Hook function triggered when the training of an epoch start.
        The function shuffle the noise dataset."""
        self.train_loader.dataset._shuffle_noise_dataset()              # shuffle noise sample among epochs
        self.train_loader.dataset.set_epoch(epoch=self.current_epoch)

    def training_step(self, batch, batch_idx) -> Dict[str, torch.Tensor]:
        """Hook function performing the training step.
        Forward the model, compute the loss function and the information for TensorBoard logger.
        
        Parameters
        ----------
        batch: torch.Tensor
            Input tensor built by "_train_collate_fn"
        
        Returns
        -------
        Dict[str, torch.Tensor]
            Dictionary containing "loss", "logits", "targets" and average "snr" of the batch
        """
        x, y, snr = batch
        logits = self.forward(x=x)
        loss = self.loss_fn(input=logits, target=y)

        train_step_output = {"loss": loss, "logits": logits, "targets": y, "snr": snr}
        return train_step_output
    
    def training_epoch_end(self, train_step_outputs) -> None:
        """Hook function triggered when training epoch ends.
        Produce the statistics for Tensorboard logger.

        Parameters
        ----------
        train_step_outputs: torch.Tensor
            List of dictionaries produced by repeating of training steps
        """
        len_train = len(self.train_loader)              # compute size of training set
        # Build the epoch logits, targets and snrs tensors from the values of each iteration. Start from the first batch iteration then concatenate the followings
        logits = train_step_outputs[0]["logits"]
        targets = train_step_outputs[0]["targets"]
        avg_snr = train_step_outputs[0]["snr"]
        for i in range(1, len_train):
            logits = torch.concat(tensors=[logits, train_step_outputs[i]["logits"]])
            targets = torch.concat(tensors=[targets, train_step_outputs[i]["targets"]])
            avg_snr += train_step_outputs[i]["snr"]
        
        loss = self.loss_fn(input=logits, target=targets)
        preds = torch.max(input=logits, dim=1).indices
        accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="micro")
        # balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="weighted")
        avg_snr /= len(train_step_outputs)

        self.log("{}_loss".format(self.task), loss, on_epoch=True, prog_bar=True, logger=True)    # save train loss on tensorboard logger
        self.log("{}_accuracy".format(self.task), accuracy, on_epoch=True, prog_bar=False, logger=True)
        # self.log("train_balanced_accuracy", balanced_accuracy, on_epoch=True, prog_bar=False, logger=True)
        self.log("train_snr", avg_snr, on_epoch=True, prog_bar=False, logger=True)

    
    def validation_step(self, batch, batch_idx) -> Dict[str, torch.Tensor]:
        """Hook function performing the validation step.
        Forward the model, compute the loss function and the information for TensorBoard logger.
        
        Parameters
        ----------
        batch: torch.Tensor
            Input tensor built by "_val_collate_fn"
        
        Returns
        -------
        Dict[str, torch.Tensor]
            Dictionary containing "logits", "targets", "snrs" of the batch
        """
        x, y, snr = batch
        logits = self.forward(x=x)

        validation_step_output = {"logits": logits, "targets": y, "snrs": snr}
        return validation_step_output

    def validation_epoch_end(self, val_step_outputs) -> None:
        """Hook function triggered when validation epoch ends.
        Produce the statistics for Tensorboard logger. The statistics are computed for each SNR pool.

        Parameters
        ----------
        val_step_outputs: torch.Tensor
            List of dictionaries produced by repeating of validation steps
        """
        len_val = len(self.val_loader)              # compute size of validation set
        # Build the epoch logits, targets and snrs tensors from the values of each iteration
        logits = val_step_outputs[0]["logits"]
        targets = val_step_outputs[0]["targets"]
        snrs = val_step_outputs[0]["snrs"]
        for i in range(1, len_val):
            logits = torch.concat(tensors=[logits, val_step_outputs[i]["logits"]])
            targets = torch.concat(tensors=[targets, val_step_outputs[i]["targets"]])
            snrs = torch.concat(tensors=[snrs, val_step_outputs[i]["snrs"]])

        # Compute average metrics
        loss = self.loss_fn(input=logits, target=targets)
        preds = torch.max(input=logits, dim=1).indices
        accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="micro")
        # balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="weighted")
        '''
        pred_reject_y = preds == (self.num_labels-1)             # '1' for reject, '0' for command
        targ_reject_y = targets == (self.num_labels-1)            # '1' for reject, '0' for command
        reject_accuracy = torchmetrics.functional.classification.binary_accuracy(preds=pred_reject_y, target=targ_reject_y)
        #'''

        #print("Validation - preds: {}, targs: {}, snr: {}, loss: {}, accuracy: {}, balanced accuracy: {}, reject accuracy: {}".format(preds.size(), targs.size(), snrs.size(), loss, accuracy, balanced_accuracy, reject_accuracy))

        # Save for TensorBoard
        self.log("val_loss", loss, on_epoch=True, prog_bar=True, logger=True)
        self.log("accuracy", accuracy, on_epoch=True, prog_bar=True, logger=True)
        '''
        self.log("balanced_accuracy", balanced_accuracy, on_epoch=True, prog_bar=False, logger=True)
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
            '''
            snr_balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=outputs[snr]["preds"], target=outputs[snr]["targs"], task="multiclass", num_classes=self.num_labels, average="weighted")
            pred_reject_y = outputs[snr]["preds"] == (self.num_labels-1)    # '1' for reject, '0' for command
            targ_reject_y = outputs[snr]["targs"] == (self.num_labels-1)    # '1' for reject, '0' for command
            snr_reject_accuracy = torchmetrics.functional.classification.binary_accuracy(preds=pred_reject_y, target=targ_reject_y)
            '''
            self.log("accuracy_{}_dB".format(snr), snr_accuracy, on_epoch=True, logger=True)
            '''
            self.log("balanced_accuracy_{}_dB".format(snr), snr_balanced_accuracy, on_epoch=True, logger=True)
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
    

    def test_step(self, batch, batch_idx, dataloader_idx=0):
        """Hook function performing the test step.
        Forward the model, compute the loss function and the information for TensorBoard logger.
        
        Parameters
        ----------
        batch: torch.Tensor
            Input tensor built by "_val_collate_fn"
        
        Returns
        -------
        Dict[str, torch.Tensor]
            Dictionary containing "logits", "targets", "snrs" of the batch
        """
        x, y, snr = batch
        logits = self.forward(x=x)

        test_step_output = {"logits": logits, "targets": y, "snrs": snr}
        return test_step_output
    
    def test_epoch_end(self, outputs):
        accuracies = list()
        for dataloader in range(len(outputs)):
            test_len = len(self.test_loaders[dataloader])              # compute size of validation set
            # Build the epoch logits, targets and snrs tensors from the values of each iteration
            logits = outputs[dataloader][0]["logits"]
            targets = outputs[dataloader][0]["targets"]
            snrs = outputs[dataloader][0]["snrs"]
            for step in range(1, test_len):
                logits = torch.concat(tensors=[logits, outputs[dataloader][step]["logits"]])
                targets = torch.concat(tensors=[targets, outputs[dataloader][step]["targets"]])
                snrs = torch.concat(tensors=[snrs, outputs[dataloader][step]["snrs"]])

            # Compute average metrics
            #loss = self.loss_fn(input=logits, target=targets)
            preds = torch.max(input=logits, dim=1).indices
            accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="micro")
    
            accuracies.append(accuracy)

        '''ADD STATITICAL ANALYSIS ON <accuracies>'''
        os.makedirs(self.settings.testing.folder, exist_ok=True)
        self.results_path = os.path.join(self.settings.testing.folder, "{}.txt".format(self.settings.logger.name))
        with open(self.results_path, 'w') as fout:
            avg = 0
            for dataloader in range(len(accuracies)):
                fout.write("Dataloader {}:\t<{:.2f}> %\n".format(dataloader, accuracies[dataloader]*100))
                avg += accuracies[dataloader]*100
            fout.write("-------------------------\n")
            fout.write("Total average:\t<{:.2f}> %\n".format(avg/len(accuracies)))
    

    def configure_callbacks(self):
        """Configure training callbacks (optimizers, checkpoint, schedulers, etc)."""
        early_stop = EarlyStopping(monitor=self.settings.training.checkpoint.metric_to_track, patience=self.settings.training.early_stop.patience, verbose=True, mode=self.settings.training.optimizer.mode, check_finite=True, check_on_train_epoch_end=False)
        lr_monitor = LearningRateMonitor(logging_interval="epoch")
        checkpoint = ModelCheckpoint(save_top_k=self.settings.training.checkpoint.save_top_k, monitor=self.settings.training.checkpoint.metric_to_track)
        return [early_stop, lr_monitor, checkpoint]
        # return [lr_monitor, checkpoint]

    def configure_optimizers(self):
        """Configure training optimizers."""
        optimizer = torch.optim.Adam(self.parameters(), lr=self.learning_rate, betas=self.settings.training.optimizer.betas, eps=self.settings.training.optimizer.eps, weight_decay=self.settings.training.optimizer.weight_decay, amsgrad=self.settings.training.optimizer.amsgrad)
        #optimizer = torch.optim.AdamW(self.parameters(), lr=self.learning_rate, betas=self.settings.training.optimizer.betas, eps=self.settings.training.optimizer.eps, weight_decay=self.settings.training.optimizer.weight_decay, amsgrad=self.settings.training.optimizer.amsgrad)
        #optimizer = torch_optimizer.NovoGrad(params=self.parameters(), lr=self.learning_rate, betas=self.settings.training.optimizer.betas, eps=self.settings.training.optimizer.eps, weight_decay=self.settings.training.optimizer.weight_decay, grad_averaging=self.settings.training.optimizer.grad_averaging, amsgrad=self.settings.training.optimizer.amsgrad)
        scheduler = ReduceLROnPlateau(optimizer=optimizer, mode=self.settings.training.optimizer.mode, patience=self.settings.training.reduce_lr_on_plateau.patience, eps=self.settings.training.optimizer.eps, verbose=True)
        optimizers_schedulers = {
            "optimizer": optimizer,
            "lr_scheduler": {
                "scheduler": scheduler,
                "monitor": self.settings.training.checkpoint.metric_to_track,
                "frequency": self.settings.training.check_val_every_n_epoch
            }
        }
        return optimizers_schedulers