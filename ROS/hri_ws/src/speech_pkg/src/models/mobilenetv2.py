from dotmap import DotMap
import torch
import torch.nn as nn
from torchvision.models import MobileNetV2, mobilenet_v2
from models.pl_backbone import PL_Backbone


class MobileNetV2_PL(PL_Backbone):

    def __init__(self, settings:DotMap, num_labels:int, loss_weights:torch.Tensor=None):
        super().__init__(settings=settings, num_labels=num_labels, loss_weights=loss_weights)
        self.save_hyperparameters()
    
    
    def set_model_parameters(self, num_labels:int):
        self.settings = self.settings
        self.task = self.settings.tasks[0].lower()
        self.num_labels = num_labels
        self.downsample = nn.Sequential(nn.Conv2d(1, 3, 3, padding=(1, 3)),
                                            nn.BatchNorm2d(3),
                                            nn.ReLU(),
                                            nn.MaxPool2d((1, 2)))
        self.model = mobilenet_v2(pretrained=False)
        model = MobileNetV2(num_classes=num_labels)
        self.model.classifier = model.classifier
        self.softmax = torch.nn.Softmax(dim=1)

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
        '''
        x = x[:, :1]  # log-Mels only
        #'''
        if self.settings.model.input.normalize:
            x = torch.nn.functional.normalize(input=x)
        x = x[:, :1]  # log-Mels only
        x = x.permute(0, 1, 3, 2).contiguous()  # Original res8 uses (time, frequency) format -> from (B, C, F, T) to (B, C, T, F)
        #'''
        x = self.downsample(x)
        return self.model.features(x)
    
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
        # Cannot use "squeeze" as batch-size can be 1
        x = nn.functional.adaptive_avg_pool2d(x, (1, 1))
        x = torch.flatten(x, 1)
        return self.model.classifier(x)