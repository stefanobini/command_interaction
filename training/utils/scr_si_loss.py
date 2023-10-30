from typing import List
import torch
import torch.nn as nn


class MT_loss(nn.Module):
    def __init__(self, tasks:List[str], weights:List[torch.Tensor]):
        super(MT_loss, self).__init__()

        self.weights = weights
        self.loss_fn = list()
        for i in range(len(tasks)):
            if tasks[i] == "snr":
                self.loss_fn.append(nn.MSELoss())
            else:
                self.loss_fn.append(nn.CrossEntropyLoss(weight=weights[i]))


    def forward(self, task_logits, task_targets):
        assert(len(task_logits) == len(task_targets))
        losses = list()
        for task in range(len(task_logits)):
            loss = self.loss_fn[task](input=task_logits[task], target=task_targets[task])
            losses.append(loss)

        outputs = torch.stack(tensors=losses)

        return outputs