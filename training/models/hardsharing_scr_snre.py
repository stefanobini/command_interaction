from dotmap import DotMap
import numpy as np
import torch
import colorama
colorama.init(autoreset=True)
from typing import List, Tuple, Dict

# from settings.MTL_conf import settings
from models.pl_mtl_backbone import PL_MTL_Backbone


class HardSharing_PL(PL_MTL_Backbone):

    def __init__(self, settings:DotMap, task_n_labels:List[int], task_loss_weights:np.float32=None):
        super().__init__(settings=settings, task_n_labels=task_n_labels, task_loss_weights=task_loss_weights)
        self.save_hyperparameters()
        

    def set_model_parameters(self, task_n_labels:List[int], task_loss_weights:np.float32=None):
        self.tasks = self.settings.tasks
        self.n_tasks = len(self.tasks)
        self.task_n_labels = task_n_labels

        embedding_size = self.settings.model.resnet8.out_channel
        self.conv0 = torch.nn.Conv2d(1, embedding_size, (3, 3), padding=(1, 1), bias=False)
        self.pool = torch.nn.AvgPool2d(self.settings.model.resnet8.pooling_size)  # flipped -- better for 80 log-Mels
        self.n_layers = n_layers = 6
        self.n_shared_layer = self.n_layers
        self.last_shared_layer_name = f'conv{self.n_shared_layer}'
        self.convs = [torch.nn.Conv2d(embedding_size, embedding_size, (3, 3), padding=1, bias=False) for _ in range(n_layers)]
        for i, conv in enumerate(self.convs):
            self.add_module(f'bn{i + 1}', torch.nn.BatchNorm2d(embedding_size, affine=False))
            self.add_module(f'conv{i + 1}', conv)
        '''For SrID
        self.speaker_features = torch.nn.Linear(embedding_size, settings.model.resnet8.speaker_embedding_size)
        self.speaker_output = torch.nn.Linear(settings.model.resnet8.speaker_embedding_size, task_n_labels[1])
        #'''
        #self.output_layers = dict()
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
        '''For SrID
        shared_embedding, speaker_embedding = self.get_embeddings(x=x) # For SrID
        return self.command_output(shared_embedding), self.speaker_output(speaker_embedding)
        #'''
        shared_embedding = self.get_embeddings(x=x)
        outputs = list()
        for task in range(self.n_tasks):
            output = getattr(self, "out_{}".format(self.tasks[task]))(shared_embedding)
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
        x = torch.mean(x, 2)      # Global Average Pooling
        '''For SrID
        speaker_embedding = self.speaker_features(x)
        return x, speaker_embedding
        #'''
        return x