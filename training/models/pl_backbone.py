from typing import Dict, List
import os
import shutil
import pandas

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


class PL_Backbone(pl.LightningModule):
    def __init__(self, settings:DotMap, num_labels:int, loss_weights:torch.Tensor=None):
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
        self.set_model_parameters(num_labels=num_labels)
        self.set_training_parameters(loss_weights=loss_weights)
        self.initialize_results_list()
        self.set_results_path()
        self.save_hyperparameters()


    def set_model_parameters(self, num_labels:int, out_channel:int):
        self.task = self.settings.tasks[0].lower()
        self.num_labels = num_labels
        self.out_channel = out_channel
        self.output = torch.nn.Linear(out_channel, num_labels)
        self.softmax = torch.nn.Softmax(dim=1)

    def set_training_parameters(self, loss_weights:torch.Tensor=None):
        self.loss_weights = loss_weights
        self.loss_fn = torch.nn.CrossEntropyLoss(weight=self.loss_weights)
        self.batch_size = self.settings.training.batch_size
        self.learning_rate = self.settings.training.lr.value
        self.snrs = list(range(self.settings.noise.min_snr, self.settings.noise.max_snr+self.settings.noise.snr_step, self.settings.noise.snr_step))

    def initialize_results_list(self):
        # Store the output of each step for analyze them at the end of the epoch
        self.train_step_outputs = list()
        self.val_step_outputs = list()
        self.test_step_outputs = list()
        for fold in range(self.settings.testing.n_folds):
            self.test_step_outputs.append(list())
        
    def set_results_path(self):
        self.results_folder = os.path.join(self.settings.logger.folder, self.settings.logger.name, self.settings.logger.version)
        self.results_file = os.path.join(self.results_folder, "real_test_results.txt") if self.settings.testing.real_data.folder else os.path.join(self.results_folder, "test_results.txt")
        if os.path.exists(self.results_file):
            os.remove(self.results_file)
        self.results_file_snr = os.path.join(self.results_folder, "test_results_snr.txt")
        if os.path.exists(self.results_file_snr):
            os.remove(self.results_file_snr)


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
        #print(Back.BLUE + "Input shape: {}".format(x.size()))
        x = self.get_embeddings(x=x)
        return self.output(x)
    
    def get_embeddings(self, x:torch.Tensor) -> torch.Tensor:
        """Extract the embedding from the input.
        Input size: (Batch, Channel, Frequency, Time)
        
        Parameters
        ----------
        x: torch.Tensor
            Input tensor
        
        Returns
        -------
        torch.Tensor
            Embedding obtained from the second-last layer of the network
        """
        return x

    def predict(self, x:torch.Tensor) -> torch.Tensor:
        x = self.forward(x)
        y = self.softmax(x)
        return y
    

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
        # x, y, _, _, snr = batch
        # x, _, y, _, snr = batch
        # x, _, _, y, snr = batch
        logits = self.forward(x=x)
        #print("TRAINING: ", torch.cuda.memory_allocated(self.device))
        loss = self.loss_fn(input=logits, target=y)

        self.train_step_outputs.append({"logits": logits.detach(),
                                        "targets": y.detach(),
                                        "snr": snr.detach()})
        return loss
    
    def on_train_epoch_end(self) -> None:
        """Hook function triggered when training epoch ends.
        Produce the statistics for Tensorboard logger.
        """
        len_train = len(self.train_loader)              # compute size of training set
        # Build the epoch logits, targets and snrs tensors from the values of each iteration. Start from the first batch iteration then concatenate the followings
        logits = self.train_step_outputs[0]["logits"]
        targets = self.train_step_outputs[0]["targets"]
        avg_snr = self.train_step_outputs[0]["snr"]
        for i in range(1, len_train):
            logits = torch.concat(tensors=[logits, self.train_step_outputs[i]["logits"]])
            targets = torch.concat(tensors=[targets, self.train_step_outputs[i]["targets"]])
            avg_snr += self.train_step_outputs[i]["snr"]
        
        loss = self.loss_fn(input=logits, target=targets)
        preds = torch.max(input=logits, dim=1).indices
        accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="micro").detach()
        # balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="weighted")
        avg_snr /= len(self.train_step_outputs)

        # Clean list of the step outputs
        self.train_step_outputs.clear()  # free memory

        self.log("{}_train_loss".format(self.task), loss, on_epoch=True, prog_bar=True, logger=True)    # save train loss on tensorboard logger
        self.log("{}_train_accuracy".format(self.task), accuracy, on_epoch=True, prog_bar=False, logger=True)
        # self.log("train_balanced_accuracy", balanced_accuracy, on_epoch=True, prog_bar=False, logger=True)
        # self.log("train_snr", avg_snr, on_epoch=True, prog_bar=False, logger=True)

    
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
        # x, y, _, _, snr = batch
        # x, _, y, _, snr = batch
        # x, _, _, y, snr = batch
        logits = self.forward(x=x)
        #print("VALIDATION: ", torch.cuda.memory_allocated(self.device))

        self.val_step_outputs.append({"logits": logits,
                                      "targets": y,
                                      "snrs": snr})

    def on_validation_epoch_end(self) -> None:
        """Hook function triggered when validation epoch ends.
        Produce the statistics for Tensorboard logger. The statistics are computed for each SNR pool.
        """
        len_val = len(self.val_loader)              # compute size of validation set
        # Build the epoch logits, targets and snrs tensors from the values of each iteration
        logits = self.val_step_outputs[0]["logits"]
        targets = self.val_step_outputs[0]["targets"]
        snrs = self.val_step_outputs[0]["snrs"]
        for i in range(1, len_val):
            logits = torch.concat(tensors=[logits, self.val_step_outputs[i]["logits"]])
            targets = torch.concat(tensors=[targets, self.val_step_outputs[i]["targets"]])
            snrs = torch.concat(tensors=[snrs, self.val_step_outputs[i]["snrs"]])
        
        # Clean list of the step outputs
        self.val_step_outputs.clear()  # free memory

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
        self.log("val_loss".format(self.task), loss, on_epoch=True, prog_bar=True, logger=True)
        self.log("{}_val_accuracy".format(self.task), accuracy, on_epoch=True, prog_bar=True, logger=True)
        '''
        self.log("balanced_accuracy", balanced_accuracy, on_epoch=True, prog_bar=False, logger=True)
        self.log("reject_accuracy", reject_accuracy, on_epoch=True, prog_bar=True, logger=True)
        '''
        ## Compute metrics for each SNR
        # Construct and fill dictionary that contains the predictions and target devided by SNR
        outputs = dict()
        for snr in self.snrs:
            outputs[snr] = {"preds": list(), "targs": list()}
        for idx in range(len(snrs)):
        #for idx in range(len_val):
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
            pred_reject_y = outputs[snr]["preds"] == (self.num_labels-1)    # '1' for reject, '0' for command
            targ_reject_y = outputs[snr]["targs"] == (self.num_labels-1)    # '1' for reject, '0' for command
            snr_reject_accuracy = torchmetrics.functional.classification.binary_accuracy(preds=pred_reject_y, target=targ_reject_y)
            
            self.log("accuracy_{}_dB".format(snr), snr_accuracy, on_epoch=True, logger=True)
            self.log("balanced_accuracy_{}_dB".format(snr), snr_balanced_accuracy, on_epoch=True, logger=True)
            self.log("reject_accuracy_{}_dB".format(snr), snr_reject_accuracy, on_epoch=True, logger=True)

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

        self.test_step_outputs[dataloader_idx].append({"logits": logits,
                                                       "targets": y,
                                                       "snrs": snr})
    
    def on_test_epoch_end(self):
        accuracies, balanced_accuracies, reject_accuracies = list(), list(), list()
        outputs = dict()
        for dataloader in range(len(self.test_step_outputs)):
            n_batch = len(self.test_loaders[dataloader])              # compute size of this dtaloader of the test set
            #print("Dataloader n. {} has  {} batches".format(dataloader, n_batch))
            # Build the epoch logits, targets and snrs tensors from the values of each iteration
            logits = self.test_step_outputs[dataloader][0]["logits"]
            targets = self.test_step_outputs[dataloader][0]["targets"]
            snrs = self.test_step_outputs[dataloader][0]["snrs"]
            for batch in range(1, n_batch):
                logits = torch.concat(tensors=[logits, self.test_step_outputs[dataloader][batch]["logits"]])
                targets = torch.concat(tensors=[targets, self.test_step_outputs[dataloader][batch]["targets"]])
                snrs = torch.concat(tensors=[snrs, self.test_step_outputs[dataloader][batch]["snrs"]])

            # Clean list of the step outputs
            self.test_step_outputs[dataloader] = list()

            # Compute average metrics
            #loss = self.loss_fn(input=logits, target=targets)
            preds = torch.max(input=logits, dim=1).indices
            accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="micro")
            balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=preds, target=targets, task="multiclass", num_classes=self.num_labels, average="weighted")
            pred_reject_y = preds == (self.num_labels-1)             # '1' for reject, '0' for command
            targ_reject_y = targets == (self.num_labels-1)            # '1' for reject, '0' for command
            reject_accuracy = torchmetrics.functional.classification.binary_accuracy(preds=pred_reject_y, target=targ_reject_y)
    
            accuracies.append(accuracy.cpu().numpy())
            balanced_accuracies.append(balanced_accuracy.cpu().numpy())
            reject_accuracies.append(reject_accuracy.cpu().numpy())

            ''' ADD SNRs Analysis
            # Construct and fill dictionary that contains the predictions and target devided by SNR
            for snr in self.snrs:
                outputs[snr] = {"logits": list(), "targets": list()}
            for idx in range(len(snrs)):
                outputs[snrs[idx].item()]["logits"].append(preds[idx])
                outputs[snrs[idx].item()]["targets"].append(targets[idx])
            
            # Convert lists in tensors
            for snr in self.snrs:    
                outputs[snr]["logits"] = torch.stack(outputs[snr]["logits"])
                outputs[snr]["targets"] = torch.stack(outputs[snr]["targets"])
            
            # Compute metrics for each SNR
            with open(self.results_file_snr, 'w') as fout:
                avg_accuracy, avg_balanced_accuracy, avg_reject_accuracy = 0, 0, 0
                snr_accuracies, snr_balanced_accuracies, snr_reject_accuracies = list(), list(), list()
                fout.write("SNR\t\t\tAccuracy\tBalanced\tReject\n" + "-"*50 + "\n")
                for snr in self.snrs:
                    snr_accuracy = torchmetrics.functional.classification.accuracy(preds=outputs[snr]["logits"], target=outputs[snr]["targets"], task="multiclass", num_classes=self.num_labels, average="micro")
                    snr_balanced_accuracy = torchmetrics.functional.classification.accuracy(preds=outputs[snr]["logits"], target=outputs[snr]["targets"], task="multiclass", num_classes=self.num_labels, average="weighted")
                    pred_reject_y = outputs[snr]["logits"] == (self.num_labels-1)    # '1' for reject, '0' for command
                    targ_reject_y = outputs[snr]["targets"] == (self.num_labels-1)    # '1' for reject, '0' for command
                    snr_reject_accuracy = torchmetrics.functional.classification.binary_accuracy(preds=pred_reject_y, target=targ_reject_y)
                    snr_accuracies.append(snr_accuracy.item())
                    snr_balanced_accuracies.append(snr_balanced_accuracy.item())
                    snr_reject_accuracies.append(snr_reject_accuracy.item())

                    fout.write("{}dB:\t\t<{:.2f}> %\t<{:.2f}> %\t<{:.2f}> %\n".format(snr, snr_accuracy*100, snr_balanced_accuracy*100, snr_reject_accuracy*100))
                    avg_accuracy += snr_accuracy*100
                    avg_balanced_accuracy += snr_balanced_accuracy*100
                    avg_reject_accuracy += snr_reject_accuracy*100
                    
                columns = ["snr", "accuracy", "balanced_accuracy", "reject_accuracy"]
                result_dict = {"snr": [snr for snr in self.snrs], "accuracy": snr_accuracies, "balanced_accuracy": snr_balanced_accuracies, "reject_accuracy": snr_reject_accuracies}
                result_df = pandas.DataFrame(data=result_dict, columns=columns)
                result_df.to_csv(path_or_buf=self.results_file_snr.replace(".txt", ".csv"), columns=columns, index=False)
                fout.write("-"*50 + "\n")
                fout.write("Total average:\t<{:.2f}> %\t<{:.2f}> %\t<{:.2f}> %\n".format(avg_accuracy/len(self.snrs), avg_balanced_accuracy/len(self.snrs), avg_reject_accuracy/len(self.snrs)))
        
            #'''
            
        #''' ADD STATITICAL ANALYSIS ON <accuracies>
        columns = ["fold", "accuracy", "balanced_accuracy", "reject_accuracy"]
        result_dict = {"fold": [i for i in range(self.settings.testing.n_folds)], "accuracy": accuracies, "balanced_accuracy": balanced_accuracies, "reject_accuracy": reject_accuracies}
        result_df = pandas.DataFrame(data=result_dict, columns=columns)
        result_df.to_csv(path_or_buf=self.results_file.replace(".txt", ".csv"), columns=columns, index=False)
        with open(self.results_file, 'w') as fout:
            fout.write("Dataloader\tAccuracy\tBalanced\tReject\n")
            avg_accuracy, avg_balanced_accuracy, avg_reject_accuracy = 0, 0, 0
            for dataloader in range(len(accuracies)):
                fout.write("Dataloader {}:\t<{:.2f}> %\t<{:.2f}> %\t<{:.2f}> %\n".format(dataloader, accuracies[dataloader]*100, balanced_accuracies[dataloader]*100, reject_accuracies[dataloader]*100))
                avg_accuracy += accuracies[dataloader]*100
                avg_balanced_accuracy += balanced_accuracies[dataloader]*100
                avg_reject_accuracy += reject_accuracies[dataloader]*100
            fout.write("-------------------------------------------------\n")
            fout.write("Total average:\t<{:.2f}> %\t<{:.2f}> %\t<{:.2f}> %\n".format(avg_accuracy/len(accuracies), avg_balanced_accuracy/len(accuracies), avg_reject_accuracy/len(accuracies)))
        #'''


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