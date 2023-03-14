import torch.nn as nn


class MT_loss(nn.Module):
    def __init__(self, scr_weight, si_weight):
        super(MT_loss, self).__init__()

        self.scr_weight, self.si_weight = scr_weight, si_weight


    def forward(self, command_logits, commands, speaker_logits, speakers):
        scr_loss_fn = nn.CrossEntropyLoss(weight=self.scr_weight)
        si_loss_fn = nn.CrossEntropyLoss(weight=self.si_weight)

        scr_loss = scr_loss_fn(input=command_logits, target=commands)
        si_loss = si_loss_fn(input=speaker_logits, target=speakers)

        return scr_loss + si_loss, scr_loss, si_loss