from dotmap import DotMap
import torch
from torchaudio.models import Conformer
from models.pl_backbone import PL_Backbone


class Conformer_PL(PL_Backbone):
    def __init__(self, settings:DotMap, num_labels:int, loss_weights:torch.Tensor=None):
        """ """        
        super().__init__(settings=settings, num_labels=num_labels, loss_weights=loss_weights)
        self.save_hyperparameters()


    def set_model_parameters(self, num_labels:int):
        input_dim:int = self.settings.input.mel.n_mels
        num_heads:int = self.settings.model.conformer.num_heads
        ffn_dim:int = self.settings.model.conformer.ffn_dim
        num_layers:int = self.settings.model.conformer.num_layers
        depthwise_conv_kernel_size:int = self.settings.model.conformer.depthwise_conv_kernel_size
        dropout:float = self.settings.model.conformer.dropout
        use_group_norm:bool = self.settings.model.conformer.use_group_norm
        convolution_first:bool = self.settings.model.conformer.convolution_first
        self.task = self.settings.task.lower()
        self.num_labels = num_labels
        out_channel = input_dim

        self.model = Conformer(input_dim=input_dim, num_heads=num_heads, ffn_dim=ffn_dim, num_layers=num_layers, depthwise_conv_kernel_size=depthwise_conv_kernel_size, dropout=dropout, use_group_norm=use_group_norm, convolution_first=convolution_first)
        self.output = torch.nn.Linear(out_channel, num_labels)
        self.softmax = torch.nn.Softmax(dim=1)
    

    def get_embeddings(self, x:torch.Tensor):
        if self.settings.model.input.normalize:
            x = torch.nn.functional.normalize(input=x)
        x = x[:, :1]  # log-Mels only
        x = x.permute(0, 1, 3, 2).contiguous()  # Original res8 uses (time, frequency) format -> from (B, C, F, T) to (B, C, T, F)
        x = torch.squeeze(input=x)
        lengths = torch.tensor([sample.size(-2) for sample in x], device=self.settings.training.device)   # build the tensor with the lenght of all sample in the batch
        y, lenghts = self.model.forward(input=x, lengths=lengths)
        y = torch.mean(input=y, dim=1).squeeze()    # Global avarange pooling along T dimension (leght of the sample)
        return y