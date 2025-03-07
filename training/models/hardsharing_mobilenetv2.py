from typing import Dict
import os
import sys
import pickle
import shutil
import numpy as np

import torch
import torch.nn as nn
import torchmetrics
from torch.optim.lr_scheduler import ReduceLROnPlateau
import pytorch_lightning as pl
from torchvision.models import MobileNetV2, mobilenet_v2
from pytorch_lightning.callbacks import EarlyStopping, ModelCheckpoint, LearningRateMonitor
from dotmap import DotMap

# import torch_optimizer

import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
from typing import Dict, List

# from settings.MTL_conf import settings
from utils.scr_si_loss import MT_loss


class HardSharing_PL(pl.LightningModule):

    def __init__(self, settings:DotMap, task_n_labels:List[int], task_loss_weights:np.float=None):
        """
        
        Methods
        -------
        forward(self, x:torch.Tensor) -> torch.Tensor
            Forward the network
        set_train_dataloader(self, dataloader:torch.utils.data.DataLoader) -> torch.utils.data.DataLoader
            Set the training loader
        set_val_dataloader(self, dataloader:torch.utils.data.DataLoader) -> torch.utils.data.DataLoader
            Set the validation loader
        set_test_dataloader(self, dataloader:torch.utils.data.DataLoader) -> torch.utils.data.DataLoader
            Set the test loader
        get_train_loader(self) -> torch.utils.data.Dataset
            Get the training loader
        get_val_loader(self) -> torch.utils.data.Dataset
            Get the validation loader
        get_test_loader(self) -> torch.utils.data.Dataset
            Get the Test loader
        on_train_epoch_start(self) -> None
            Hook function triggered when the training of an epoch start. The function shuffle the noise dataset
        training_step(self, batch, batch_idx) -> torch.Tensor
            Hook function performing the training step. Forward the model, compute the loss function and the information for TensorBoard logger.
        training_epoch_end(self, train_step_outputs) -> None
            Hook function triggered when training epoch ends.
        Produce the statistics for Tensorboard logger.
        validation_step(self, batch, batch_idx) -> Dict[str, torch.Tensor]
            Hook function performing the validation step. Forward the model, compute the loss function and the information for TensorBoard logger.
        validation_epoch_end(self, val_step_outputs) -> None
            Hook function triggered when validation epoch ends. Produce the statistics for Tensorboard logger. The statistics are computed for each SNR pool.
        test_step(self, batch, batch_idx)
            Hook function performing the test step
        configure_callbacks(self)
            Configure training callbacks (optimizers, checkpoint, schedulers, etc)
        configure_optimizers(self)
            Configure training optimizers
        """
        super().__init__()

        self.downsample = nn.Sequential(nn.Conv2d(1, 3, 3, padding=(1, 3)),
                                        nn.BatchNorm2d(3),
                                        nn.ReLU(),
                                        nn.MaxPool2d((1, 2)))
        self.model = mobilenet_v2(pretrained=False)
        model = MobileNetV2(num_classes=task_n_labels[0])
        self.model.classifier = model.classifier

        self.command_output = torch.nn.Linear(self.model.last_channel, task_n_labels[0])
        self.speaker_features = torch.nn.Linear(self.model.last_channel, settings.model.mobilenetv2.speaker_embedding_size)
        self.speaker_output = torch.nn.Linear(settings.model.mobilenetv2.speaker_embedding_size, task_n_labels[1])
        # self.softmax = torch.nn.Softmax(dim=1)

        self.settings = settings
        self.n_tasks = 2    # SCR and SI
        self.grad_norm = self.settings.training.loss.type == "grad_norm"
        self.loss_weights = torch.nn.Parameter(data=torch.ones(self.n_tasks), requires_grad=self.grad_norm)
        self.loss_fn = MT_loss(weights1=task_loss_weights[0], weights2=task_loss_weights[1])
        
        self.task_n_labels = task_n_labels
        self.batch_size = settings.training.batch_size
        self.learning_rate = settings.training.lr.value
        self.snrs = list(range(settings.noise.min_snr, settings.noise.max_snr+settings.noise.snr_step, settings.noise.snr_step))

        self.save_hyperparameters()
        

    def forward(self, x:torch.Tensor) -> torch.Tensor:
        """Forward the network.
        Input size: (Batch, Channel, Frequency, Time)
        
        
        Parameters
        ----------
        x: torch.Tensor
            Input tensor
        
        Returns
        -------
        torch.Tensor
            Logit tensor
        """
        if self.settings.model.input.normalize:
            x = torch.nn.functional.normalize(input=x)

        x = x[:, :1]  # log-Mels only
        x = self.downsample(x)

        features_map = self.model.features(x)
        # Cannot use "squeeze" as batch-size can be 1
        features_map = nn.functional.adaptive_avg_pool2d(features_map, (1, 1))
        shared_embeddings = torch.flatten(features_map, 1)
        speaker_embeddings = self.speaker_features(shared_embeddings)

        return self.command_output(shared_embeddings), self.speaker_output(speaker_embeddings)


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
    

    def get_last_shared_layer(self):
        return self.model.features


    def on_train_start(self) -> None:
        if not self.settings.training.lr.auto_find:
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
        x, speakers, commands, snr = batch
        command_logits, speaker_logits = self.forward(x=x)

        self.losses = self.loss_fn(logits1=command_logits, targets1=commands, logits2=speaker_logits, targets2=speakers)
        weighted_task_loss = torch.mul(self.loss_weights, self.losses)
        loss = torch.sum(weighted_task_loss)

        # initialize the initial loss L(0) if t=0
        if self.current_epoch == 0 and self.grad_norm:
            # set L(0) to the Log(C) because we use CrossEntropy for both the tasks
            self.initial_task_loss = np.log(self.task_n_labels)

        train_step_output = {"loss": loss,
                             "command_logits": command_logits.detach(),
                             "speaker_logits": speaker_logits.detach(),
                             "commands": commands.detach(),
                             "speakers": speakers.detach(),
                             "snr": snr}
        return train_step_output
    

    def backward(self, loss, optimizer, optimizer_idx):
        # do a custom way of backward
        # optimizer.zero_grad(set_to_none=True)
        loss.backward(retain_graph=self.grad_norm)

    def on_after_backward(self):
        ######################
        # GradNorm algorithm #
        ######################
        if self.grad_norm:
            self.loss_weights.grad.data *= 0.0

            # get layer of shared weights
            shared_weights = self.get_last_shared_layer()

            # get the gradient norms for each of the tasks
            # G^{(i)}_w(t) 
            norms = list()
            for i in range(len(self.losses)):
                # get the gradient of this task loss with respect to the shared parameters
                gygw = torch.autograd.grad(self.losses[i], shared_weights.parameters(), retain_graph=True)
                # compute the norm
                norms.append(torch.norm(torch.mul(self.loss_weights[i], gygw[0])))
            norms = torch.stack(norms)

            # compute the inverse training rate r_i(t)
            loss_ratio = self.losses.data.cpu().numpy() / self.initial_task_loss
            # r_i(t)
            inverse_train_rate = loss_ratio / np.mean(loss_ratio)

            # compute the mean norm \tilde{G}_w(t)
            mean_norm = np.mean(norms.data.cpu().numpy())

            constant_term = torch.tensor(mean_norm * (inverse_train_rate ** self.settings.training.loss.grad_norm.alpha), requires_grad=False).cuda()
            #print('Constant term: {}'.format(constant_term))
            # this is the GradNorm loss itself
            grad_norm_loss = torch.sum(torch.abs(norms - constant_term))
            #print('GradNorm loss {}'.format(grad_norm_loss))
            '''
            grad_loss = torch.nn.L1Loss(reduction="sum")
            grad_norm_loss = 0
            for loss_index in range (0, len(self.losses)):
                grad_norm_loss = torch.add(grad_norm_loss, grad_loss(norms[loss_index], constant_term[loss_index]))
            '''

            # compute the gradient for the weights
            self.loss_weights.grad = torch.autograd.grad(grad_norm_loss, self.loss_weights)[0]


    def on_train_batch_end(self, out, batch, batch_idx):
        '''
        thresh = 0.001
        self.loss_weights.data = torch.where(self.loss_weights > thresh, self.loss_weights.data, thresh)
        '''
        # Clean all the gradient
        # self.loss_weights.
        # Check memoty occupation
        # print(Back.YELLOW + "CUDA memory occupation: {:.2f}".format(torch.cuda.memory_allocated("cuda:3")/1e9))

    def training_epoch_end(self, train_step_outputs) -> None:
        """Hook function triggered when training epoch ends.
        Produce the statistics for Tensorboard logger.

        Parameters
        ----------
        train_step_outputs: torch.Tensor
            List of dictionaries produced by repeating of training steps
        """
        if self.grad_norm:
            # renormalize (for GradNorm algorithm)
            normalize_coeff = self.n_tasks / torch.sum(self.loss_weights.data, dim=0)
            self.loss_weights.data *= normalize_coeff

        len_train = len(self.train_loader)              # compute size of training set
        # Build the epoch logits, targets and snrs tensors from the values of each iteration. Start from the first batch iteration then concatenate the followings
        command_logits = train_step_outputs[0]["command_logits"]
        speaker_logits = train_step_outputs[0]["speaker_logits"]
        commands = train_step_outputs[0]["commands"]
        speakers = train_step_outputs[0]["speakers"]
        avg_snr = train_step_outputs[0]["snr"]
        for i in range(1, len_train):
            command_logits = torch.concat(tensors=[command_logits, train_step_outputs[i]["command_logits"]])
            commands = torch.concat(tensors=[commands, train_step_outputs[i]["commands"]])
            speaker_logits = torch.concat(tensors=[speaker_logits, train_step_outputs[i]["speaker_logits"]])
            speakers = torch.concat(tensors=[speakers, train_step_outputs[i]["speakers"]])
            avg_snr += train_step_outputs[i]["snr"]
        
        losses = self.loss_fn(logits1=command_logits, targets1=commands, logits2=speaker_logits, targets2=speakers)
        weighted_task_loss = torch.mul(self.loss_weights, losses)
        loss = torch.sum(weighted_task_loss)
        scr_loss, si_loss = losses[0], losses[1]
  
        command_predictions = torch.max(input=command_logits, dim=1).indices
        speaker_predictions = torch.max(input=speaker_logits, dim=1).indices
        command_accuracy = torchmetrics.functional.classification.accuracy(preds=command_predictions, target=commands, task="multiclass", num_classes=self.task_n_labels[0], average="micro")
        speaker_accuracy = torchmetrics.functional.classification.accuracy(preds=speaker_predictions, target=speakers, task="multiclass", num_classes=self.task_n_labels[1], average="micro")
        # command_balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=command_predictions, target=commands, task="multiclass", num_classes=self.num_label1, average="weighted")
        # speaker_balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=speaker_predictions, target=speakers, task="multiclass", num_classes=self.num_label2, average="weighted")
        avg_snr /= len(train_step_outputs)

        # self.logger.experiment.add_scalars("train_accuracy", {"command": command_accuracy, "speaker": speaker_accuracy}, global_step=self.global_step)
        # self.logger.experiment.add_scalars("train_accuracy", {"train_loss": loss, "command": scr_loss, "speaker": si_loss}, global_step=self.global_step)
        self.log("train_loss", loss, on_epoch=True, prog_bar=True, logger=True)    # save train loss on tensorboard logger
        self.log("scr_train_loss", scr_loss, on_epoch=True, prog_bar=False, logger=True)    # save train loss on tensorboard logger
        self.log("scr_train_accuracy", command_accuracy, on_epoch=True, prog_bar=False, logger=True)
        self.log("scr_loss_weight", self.loss_weights[0].data, on_epoch=True, prog_bar=True, logger=True)
        # self.log("train_balanced_accuracy", balanced_accuracy, on_epoch=True, prog_bar=False, logger=True)
        self.log("si_train_loss", si_loss, on_epoch=True, prog_bar=False, logger=True)    # save train loss on tensorboard logger
        self.log("si_train_accuracy", speaker_accuracy, on_epoch=True, prog_bar=False, logger=True)
        self.log("si_loss_weight", self.loss_weights[1].data, on_epoch=True, prog_bar=True, logger=True)
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
        x, speakers, commands, snr = batch
        command_logits, speaker_logits = self.forward(x=x)

        validation_step_output = {"command_logits": command_logits,
                                  "speaker_logits": speaker_logits,
                                  "commands": commands,
                                  "speakers": speakers,
                                  "snrs": snr}
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
        command_logits = val_step_outputs[0]["command_logits"]
        speaker_logits = val_step_outputs[0]["speaker_logits"]
        commands = val_step_outputs[0]["commands"]
        speakers = val_step_outputs[0]["speakers"]
        snrs = val_step_outputs[0]["snrs"]
        for i in range(1, len_val):
            command_logits = torch.concat(tensors=[command_logits, val_step_outputs[i]["command_logits"]])
            commands = torch.concat(tensors=[commands, val_step_outputs[i]["commands"]])
            speaker_logits = torch.concat(tensors=[speaker_logits, val_step_outputs[i]["speaker_logits"]])
            speakers = torch.concat(tensors=[speakers, val_step_outputs[i]["speakers"]])
            snrs = torch.concat(tensors=[snrs, val_step_outputs[i]["snrs"]])

        # Compute average metrics
        losses = self.loss_fn(logits1=command_logits, targets1=commands, logits2=speaker_logits, targets2=speakers)
        weighted_task_loss = torch.mul(self.loss_weights, losses)
        loss = torch.sum(weighted_task_loss)
        scr_loss, si_loss = losses[0], losses[1]

        command_predictions = torch.max(input=command_logits, dim=1).indices
        speaker_predictions = torch.max(input=speaker_logits, dim=1).indices
        command_accuracy = torchmetrics.functional.classification.accuracy(preds=command_predictions, target=commands, task="multiclass", num_classes=self.task_n_labels[0], average="micro")
        speaker_accuracy = torchmetrics.functional.classification.accuracy(preds=speaker_predictions, target=speakers, task="multiclass", num_classes=self.task_n_labels[1], average="micro")
        # balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="weighted")
        '''
        pred_reject_y = preds == (self.num_labels-1)             # '1' for reject, '0' for command
        targ_reject_y = targets == (self.num_labels-1)            # '1' for reject, '0' for command
        reject_accuracy = torchmetrics.functional.classification.binary_accuracy(preds=pred_reject_y, target=targ_reject_y)
        #'''

        #print("Validation - preds: {}, targs: {}, snr: {}, loss: {}, accuracy: {}, balanced accuracy: {}, reject accuracy: {}".format(preds.size(), targs.size(), snrs.size(), loss, accuracy, balanced_accuracy, reject_accuracy))

        # Save for TensorBoard
        self.log("val_loss", loss, on_epoch=True, prog_bar=True, logger=True)    # save train loss on tensorboard logger
        self.log("scr_val_loss", scr_loss, on_epoch=True, prog_bar=False, logger=True)    # save train loss on tensorboard logger
        self.log("scr_val_accuracy", command_accuracy, on_epoch=True, prog_bar=False, logger=True)
        # self.log("train_balanced_accuracy", balanced_accuracy, on_epoch=True, prog_bar=False, logger=True)
        self.log("si_val_loss", si_loss, on_epoch=True, prog_bar=False, logger=True)    # save train loss on tensorboard logger
        self.log("si_val_accuracy", speaker_accuracy, on_epoch=True, prog_bar=False, logger=True)
        
    
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