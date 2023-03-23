import torch
import torch.nn as nn


class GradNorm(nn.Module):
    def __init__(self, weights1, weights2):
        super(GradNorm, self).__init__()

        self.weights1, self.weights2 = weights1, weights2
        self.loss_fn1 = nn.CrossEntropyLoss(weight=self.weights1)
        self.loss_fn2 = nn.CrossEntropyLoss(weight=self.weights2)


    def forward(self, logits1, targets1, logits2, targets2):

        loss1 = self.loss_fn1(input=logits1, target=targets1)
        loss2 = self.loss_fn2(input=logits2, target=targets2)

        outputs = torch.stack(tensors=(loss1, loss2))

        return outputs