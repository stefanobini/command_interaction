import torch
import torch.nn as nn


class MT_loss(nn.Module):
    def __init__(self, weights1, weights2):
        super(MT_loss, self).__init__()

        self.weights1, self.weights2 = weights1, weights2
        self.loss_fn1 = nn.CrossEntropyLoss(weight=self.weights1)
        self.loss_fn2 = nn.CrossEntropyLoss(weight=self.weights2)
        # self.coefficient1, self.coefficient2 = coefficient1, coefficient2


    def forward(self, logits1, targets1, logits2, targets2):

        loss1 = self.loss_fn1(input=logits1, target=targets1)
        loss2 = self.loss_fn2(input=logits2, target=targets2)
        # total_loss = self.coefficient1*loss1 + self.coefficient2*loss2

        outputs = torch.stack(tensors=(loss1, loss2))

        return outputs