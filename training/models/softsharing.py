from typing import Dict, Tuple
import os
import pickle
import shutil
import numpy as np
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
from typing import Dict, List

# from settings.MTL_conf import settings
from utils.scr_si_loss import MT_loss
from models.pl_mtl_backbone import PL_MTL_Backbone


class SoftSharing_PL(PL_MTL_Backbone):

    def __init__(self, settings:DotMap, task_n_labels:List[int], task_loss_weights:np.float32=None):
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
        super().__init__(settings=settings, task_n_labels=task_n_labels, task_loss_weights=task_loss_weights)
        self.save_hyperparameters()

    def set_model_parameters(self, task_n_labels:List[int], task_loss_weights:np.float32=None):
        self.tasks = self.settings.tasks
        self.n_tasks = len(self.tasks)
        self.task_n_labels = task_n_labels

        embedding_size = self.settings.model.resnet8.out_channel
        self.n_layers = n_layers = 6
        self.n_shared_layer = n_shared_layer = 3
        self.n_branched_layer = n_branched_layer = n_layers - n_shared_layer

        self.shared_conv0 = torch.nn.Conv2d(1, embedding_size, (3, 3), padding=(1, 1), bias=False)
        self.shared_pool = torch.nn.AvgPool2d(self.settings.model.resnet8.pooling_size)  # flipped -- better for 80 log-Mels
        self.shared_convs = [torch.nn.Conv2d(embedding_size, embedding_size, (3, 3), padding=1, bias=False) for _ in range(n_shared_layer)]
        for i, conv in enumerate(self.shared_convs):
            self.add_module(f'shared_bn{i + 1}', torch.nn.BatchNorm2d(embedding_size, affine=False))
            self.add_module(f'shared_conv{i + 1}', conv)
        #self.add_module('{}_convs'.format(self.tasks[task]), [torch.nn.Conv2d(embedding_size, embedding_size, (3, 3), padding=1, bias=False) for _ in range(n_branched_layer)])
        for task in range(self.n_tasks):
            self.branchi_convs = [torch.nn.Conv2d(embedding_size, embedding_size, (3, 3), padding=1, bias=False) for _ in range(n_branched_layer)]
            for i, conv in enumerate(self.branchi_convs):
                self.add_module('{}_bn{}'.format(self.tasks[task], i+1), torch.nn.BatchNorm2d(embedding_size, affine=False))
                self.add_module('{}_conv{}'.format(self.tasks[task], i+1), conv)
        for task in range(self.n_tasks):
            self.add_module("out_{}".format(self.tasks[task]), torch.nn.Linear(embedding_size, task_n_labels[task]))
        # self.softmax = torch.nn.Softmax(dim=1)
        

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
        shared_embeddings = self.get_embeddings(x=x)
        outputs = list()
        for task in range(self.n_tasks):
            output = getattr(self, "out_{}".format(self.tasks[task]))(shared_embeddings[task])
            outputs.append(output)
        return outputs

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

        embeddings = list()
        for task in range(self.n_tasks):
            old_xi = old_x
            xi = x
            yi = y
            for i in range(1, self.n_branched_layer):
                yi = torch.nn.functional.relu(getattr(self, '{}_conv{}'.format(self.tasks[task], i))(xi))
                if i % 2 == 0:
                    xi = yi + old_xi
                    old_xi = xi
                else:
                    xi = yi
                xi = getattr(self, '{}_bn{}'.format(self.tasks[task], i))(xi)
            xi = xi.view(xi.size(0), xi.size(1), -1)  # shape: (batch, feats, o3)
            xi = torch.mean(xi, 2)      # Global Average Pooling
            embeddings.append(xi)
        return embeddings
    
    def predict_srid(self, x:torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        shared_embedding, speaker_embedding = self.get_embeddings(x=x)
        return self.branch1_output(shared_embedding), speaker_embedding