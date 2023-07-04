from dotmap import DotMap
import torch
from models.pl_backbone import PL_Backbone


class ResNet8_PL(PL_Backbone):
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
        super().__init__(settings=settings, num_labels=num_labels, loss_weights=loss_weights)
        self.save_hyperparameters()
        
    def set_model_parameters(self, num_labels:int):
        self.task = self.settings.tasks[0].lower()
        out_channel = self.settings.model.resnet8.out_channel
        self.num_labels = num_labels

        self.conv0 = torch.nn.Conv2d(1, out_channel, (3, 3), padding=(1, 1), bias=False)
        self.pool = torch.nn.AvgPool2d(self.settings.model.resnet8.pooling_size)  # flipped -- better for 80 log-Mels

        self.n_layers = n_layers = 6
        self.convs = [torch.nn.Conv2d(out_channel, out_channel, (3, 3), padding=1, bias=False) for _ in range(n_layers)]
        for i, conv in enumerate(self.convs):
            self.add_module(f'bn{i + 1}', torch.nn.BatchNorm2d(out_channel, affine=False))
            self.add_module(f'conv{i + 1}', conv)
        self.output = torch.nn.Linear(out_channel, num_labels)
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
        return x

    def predict_srid(self, x:torch.Tensor) -> torch.Tensor:
        """Return the embedding representing the speaker of the sample."""
        return self.get_embeddings(x=x)