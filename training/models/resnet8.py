import torch
import pytorch_lightning as pl

import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore

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

        self.loss_fn = torch.nn.CrossEntropyLoss()
        self.batch_size = settings.train.batch_size
        self.learning_rate = settings.training.lr


    def forward(self, x:torch.FloatTensor) -> torch.FloatTensor:
        """ """

        x = x[:, :1]  # log-Mels only
        x = x.permute(0, 1, 3, 2).contiguous()  # Original res8 uses (time, frequency) format
        
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


    def training_step(self, batch, batch_idx):
        """ """

        x, y = batch
        y_pred = self.forward(x=x)
        loss = self.loss_fn(y_pred, y)
        self.log("train_loss", loss)

        return loss

    
    def validation_step(self, batch, batch_idx):
        """ """
        
        x, y = batch
        y_pred = self.forward(x=x)
        loss = self.loss_fn(y_pred, y)
        self.log("val_loss", loss)
    

    def test_step(self, batch, batch_idx):
        """ """
        
        x, y = batch
        y_pred = self.forward(x=x)
        loss = self.loss_fn(y_pred, y)
        self.log("test_loss", loss)
    

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=self.learning_rate)
        return optimizer