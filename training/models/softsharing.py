from typing import Dict, Tuple
import os
import pickle
import shutil
import numpy as np
import torch
import torchmetrics
from torch.optim.lr_scheduler import ReduceLROnPlateau
import pytorch_lightning as pl
from pytorch_lightning.callbacks import EarlyStopping, ModelCheckpoint, LearningRateMonitor
from dotmap import DotMap

# import torch_optimizer

import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
from typing import Dict, List

# from settings.MTL_conf import settings
from utils.scr_si_loss import MT_loss


class SoftSharing_PL(pl.LightningModule):

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
        training_epoch_end(self) -> None
            Hook function triggered when training epoch ends.
        Produce the statistics for Tensorboard logger.
        validation_step(self, batch, batch_idx) -> Dict[str, torch.Tensor]
            Hook function performing the validation step. Forward the model, compute the loss function and the information for TensorBoard logger.
        validation_epoch_end(self) -> None
            Hook function triggered when validation epoch ends. Produce the statistics for Tensorboard logger. The statistics are computed for each SNR pool.
        test_step(self, batch, batch_idx)
            Hook function performing the test step
        configure_callbacks(self)
            Configure training callbacks (optimizers, checkpoint, schedulers, etc)
        configure_optimizers(self)
            Configure training optimizers
        """
        super().__init__()

        self.settings = settings
        embedding_size = settings.model.resnet8.out_channel

        self.shared_conv0 = torch.nn.Conv2d(1, embedding_size, (3, 3), padding=(1, 1), bias=False)
        self.shared_pool = torch.nn.AvgPool2d(settings.model.resnet8.pooling_size)  # flipped -- better for 80 log-Mels

        self.n_layers = n_layers = 6
        self.n_shared_layer = n_shared_layer = 3
        self.n_branched_layer = n_branched_layer = n_layers - n_shared_layer
        self.shared_convs = [torch.nn.Conv2d(embedding_size, embedding_size, (3, 3), padding=1, bias=False) for _ in range(n_shared_layer)]
        for i, conv in enumerate(self.shared_convs):
            self.add_module(f'shared_bn{i + 1}', torch.nn.BatchNorm2d(embedding_size, affine=False))
            self.add_module(f'shared_conv{i + 1}', conv)
        self.branch1_convs = [torch.nn.Conv2d(embedding_size, embedding_size, (3, 3), padding=1, bias=False) for _ in range(n_branched_layer)]
        for i, conv in enumerate(self.branch1_convs):
            self.add_module(f'branch1_bn{i + 1}', torch.nn.BatchNorm2d(embedding_size, affine=False))
            self.add_module(f'branch1_conv{i + 1}', conv)
        self.branch2_convs = [torch.nn.Conv2d(embedding_size, embedding_size, (3, 3), padding=1, bias=False) for _ in range(n_branched_layer)]
        for i, conv in enumerate(self.branch2_convs):
            self.add_module(f'branch2_bn{i + 1}', torch.nn.BatchNorm2d(embedding_size, affine=False))
            self.add_module(f'branch2_conv{i + 1}', conv)
        self.branch1_output = torch.nn.Linear(embedding_size, task_n_labels[0])
        self.branch2_output = torch.nn.Linear(embedding_size, task_n_labels[1])
        # self.softmax = torch.nn.Softmax(dim=1)

        self.n_tasks = 2    # SCR and SI
        self.grad_norm = self.settings.training.loss.type == "grad_norm"
        self.loss_weights = torch.nn.Parameter(data=torch.ones(self.n_tasks), requires_grad=self.grad_norm)
        self.loss_fn = MT_loss(weights1=task_loss_weights[0], weights2=task_loss_weights[1])
        
        self.task_n_labels = task_n_labels
        self.batch_size = settings.training.batch_size
        self.learning_rate = settings.training.lr.value
        self.snrs = list(range(settings.noise.min_snr, settings.noise.max_snr+settings.noise.snr_step, settings.noise.snr_step))

        # Store the output of each step for analyze them at the end of the epoch
        self.train_step_outputs = list()
        self.val_step_outputs = list()
        self.test_step_outputs = list()

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
        #print(Back.BLUE + "ResNet - input shape: {}".format(x.size()))
        embedding1, embedding2 = self.get_embeddings(x=x)
        return self.branch1_output(embedding1), self.branch2_output(embedding2)

    def get_embeddings(self, x:torch.Tensor) -> torch.Tensor:
        """Extract the shared and speaker embeddings from the input.
        Input size: (Batch, Channel, Frequency, Time)
        
        Parameters
        ----------
        x: torch.Tensor
            Input tensor
        
        Returns
        -------
        torch.Tensor
            Shared embedding obtained from the shared part of the multitask network
        torch.Tensor
            Speaker embedding obtained from the second-last layer of the speaker branch
        """
        if self.settings.model.input.normalize:
            x = torch.nn.functional.normalize(input=x)

        x = x[:, :1]  # log-Mels only
        x = x.permute(0, 1, 3, 2).contiguous()  # Original res8 uses (time, frequency) format -> from (B, C, F, T) to (B, C, T, F)
        
        # Shared network
        for i in range(self.n_shared_layer + 1):
            y = torch.nn.functional.relu(getattr(self, f'shared_conv{i}')(x))
            if i == 0:
                if hasattr(self, 'shared_pool'):
                    y = self.shared_pool(y)
                old_x = y
            if i > 0 and i % 2 == 0:
                x = y + old_x
                old_x = x
            else:
                x = y
            if i > 0:
                x = getattr(self, f'shared_bn{i}')(x)

        # Branch 1
        old_x1 = old_x
        x1 = x
        y1 = y
        for i in range(1, self.n_branched_layer):
            y1 = torch.nn.functional.relu(getattr(self, f'branch1_conv{i}')(x1))
            if i % 2 == 0:
                x1 = y1 + old_x1
                old_x1 = x1
            else:
                x1 = y1
            x1 = getattr(self, f'branch1_bn{i}')(x1)
        x1 = x1.view(x1.size(0), x1.size(1), -1)  # shape: (batch, feats, o3)
        x1 = torch.mean(x1, 2)      # Global Average Pooling

        # Branch 2
        old_x2 = old_x
        x2 = x
        y2 = y
        for i in range(1, self.n_branched_layer):
            y2 = torch.nn.functional.relu(getattr(self, f'branch2_conv{i}')(x2))
            if i % 2 == 0:
                x2 = y2 + old_x2
                old_x2 = x2
            else:
                x2 = y2
            x2 = getattr(self, f'branch2_bn{i}')(x2)
        x2 = x1.view(x2.size(0), x2.size(1), -1)  # shape: (batch, feats, o3)
        x2 = torch.mean(x2, 2)      # Global Average Pooling
        return x1, x2
    
    def predict_srid(self, x:torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        shared_embedding, speaker_embedding = self.get_embeddings(x=x)
        return self.branch1_output(shared_embedding), speaker_embedding


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
        return getattr(self, f'shared_conv{self.n_shared_layer}')


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
        x, speakers, commands, snr = batch
        command_logits, speaker_logits = self.forward(x=x)

        self.losses = self.loss_fn(logits1=command_logits, targets1=commands, logits2=speaker_logits, targets2=speakers)
        weighted_task_loss = torch.mul(self.loss_weights, self.losses)
        loss = torch.sum(weighted_task_loss)

        # initialize the initial loss L(0) if t=0
        if self.current_epoch == 0 and self.grad_norm:
            # set L(0) to the Log(C) because we use CrossEntropy for both the tasks
            self.initial_task_loss = np.log(self.task_n_labels)

        self.train_step_outputs.append({"command_logits": command_logits.detach(),
                                            "speaker_logits": speaker_logits.detach(),
                                            "commands": commands.detach(),
                                            "speakers": speakers.detach(),
                                            "snr": snr.detach()})
        return loss


    def backward(self, loss):
        # do a custom way of backward
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

            constant_term = torch.tensor(mean_norm * (inverse_train_rate ** self.settings.training.loss.grad_norm.alpha), requires_grad=False).cuda(self.settings.training.device)
            # this is the GradNorm loss itself
            grad_norm_loss = torch.sum(torch.abs(norms - constant_term))

            # compute the gradient for the weights
            self.loss_weights.grad = torch.autograd.grad(grad_norm_loss, self.loss_weights)[0]


    def on_train_batch_end(self, out, batch, batch_idx):
        '''
        thresh = 0.001
        self.loss_weights.data = torch.where(self.loss_weights > thresh, self.loss_weights.data, thresh)
        '''
        if self.grad_norm:
            # renormalize before use the weight for the validation loss
            normalize_coeff = self.n_tasks / torch.sum(self.loss_weights.data, dim=0)
            self.loss_weights.data *= normalize_coeff
            self.loss_weights.data.clamp_(min=0.1, max=2.0)
        
        # Clean all the gradient
        # self.loss_weights.
        # Check memoty occupation
        # print(Back.YELLOW + "CUDA memory occupation: {:.2f}".format(torch.cuda.memory_allocated("cuda:3")/1e9))

    def on_train_epoch_end(self) -> None:
        """Hook function triggered when training epoch ends.
        Produce the statistics for Tensorboard logger.
        """
        len_train = len(self.train_loader)              # compute size of training set
        # Build the epoch logits, targets and snrs tensors from the values of each iteration. Start from the first batch iteration then concatenate the followings
        command_logits = self.train_step_outputs[0]["command_logits"]
        speaker_logits = self.train_step_outputs[0]["speaker_logits"]
        commands = self.train_step_outputs[0]["commands"]
        speakers = self.train_step_outputs[0]["speakers"]
        avg_snr = self.train_step_outputs[0]["snr"]
        for i in range(1, len_train):
            command_logits = torch.concat(tensors=[command_logits, self.train_step_outputs[i]["command_logits"]])
            commands = torch.concat(tensors=[commands, self.train_step_outputs[i]["commands"]])
            speaker_logits = torch.concat(tensors=[speaker_logits, self.train_step_outputs[i]["speaker_logits"]])
            speakers = torch.concat(tensors=[speakers, self.train_step_outputs[i]["speakers"]])
            avg_snr += self.train_step_outputs[i]["snr"]
        
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
        avg_snr /= len(self.train_step_outputs)

        # Clean list of the step outputs
        self.train_step_outputs.clear()  # free memory

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

        self.val_step_outputs.append({"command_logits": command_logits,
                                             "speaker_logits": speaker_logits,
                                             "commands": commands,
                                             "speakers": speakers,
                                             "snrs": snr})

    def on_validation_epoch_end(self) -> None:
        """Hook function triggered when validation epoch ends.
        Produce the statistics for Tensorboard logger. The statistics are computed for each SNR pool.
        """
        len_val = len(self.val_loader)              # compute size of validation set
        # Build the epoch logits, targets and snrs tensors from the values of each iteration
        command_logits = self.val_step_outputs[0]["command_logits"]
        speaker_logits = self.val_step_outputs[0]["speaker_logits"]
        commands = self.val_step_outputs[0]["commands"]
        speakers = self.val_step_outputs[0]["speakers"]
        snrs = self.val_step_outputs[0]["snrs"]
        for i in range(1, len_val):
            command_logits = torch.concat(tensors=[command_logits, self.val_step_outputs[i]["command_logits"]])
            commands = torch.concat(tensors=[commands, self.val_step_outputs[i]["commands"]])
            speaker_logits = torch.concat(tensors=[speaker_logits, self.val_step_outputs[i]["speaker_logits"]])
            speakers = torch.concat(tensors=[speakers, self.val_step_outputs[i]["speakers"]])
            snrs = torch.concat(tensors=[snrs, self.val_step_outputs[i]["snrs"]])
        
        # Clean list of the step outputs
        self.val_step_outputs.clear()  # free memory

        # Compute average metrics
        losses = self.loss_fn(logits1=command_logits, targets1=commands, logits2=speaker_logits, targets2=speakers)
        weighted_task_loss = torch.mul(self.loss_weights, losses)
        gradNorm_loss = torch.sum(weighted_task_loss)
        loss = torch.sum((losses))
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
        self.log("val_gradNorm_loss", gradNorm_loss, on_epoch=True, prog_bar=False, logger=True)    # save train loss on tensorboard logger
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

        self.test_step_outputs.append({"logits": logits,
                                       "targets": y,
                                       "snrs": snr})
    
    def on_test_epoch_end(self):
        accuracies = list()
        for dataloader in range(len(self.test_step_outputs)):
            test_len = len(self.test_loaders[dataloader])              # compute size of validation set
            # Build the epoch logits, targets and snrs tensors from the values of each iteration
            logits = self.test_step_outputs[dataloader][0]["logits"]
            targets = self.test_step_outputs[dataloader][0]["targets"]
            snrs = self.test_step_outputs[dataloader][0]["snrs"]
            for step in range(1, test_len):
                logits = torch.concat(tensors=[logits, self.test_step_outputs[dataloader][step]["logits"]])
                targets = torch.concat(tensors=[targets, self.test_step_outputs[dataloader][step]["targets"]])
                snrs = torch.concat(tensors=[snrs, self.test_step_outputs[dataloader][step]["snrs"]])

            # Clean list of the step outputs
            self.test_step_outputs.clear()   # free memory

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
        monitored_metric = "val_gradNorm_loss" if self.settings.training.loss.type == "grad_norm" and self.settings.task == "SCR_SI" else self.settings.training.checkpoint.metric_to_track
        early_stop = EarlyStopping(monitor=monitored_metric, patience=self.settings.training.early_stop.patience, verbose=True, mode=self.settings.training.optimizer.mode, check_finite=True, check_on_train_epoch_end=False)
        lr_monitor = LearningRateMonitor(logging_interval="epoch")
        checkpoint = ModelCheckpoint(save_top_k=self.settings.training.checkpoint.save_top_k, monitor=self.settings.training.checkpoint.metric_to_track)
        return [early_stop, lr_monitor, checkpoint]

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