import os
import sys
from tqdm import tqdm

import torch
import torchaudio

from utils.dataloaders import MiviaDataset, _custom_collate_fn
from settings.conf_1 import settings


########################
### LOAD THE SUBSETS ###
########################
train_set = MiviaDataset(subset="training")
valid_set = MiviaDataset(subset="validation")
test_set = MiviaDataset(subset="testing")
print("The dataset is splitted in:\n- TRAINING samples:\t{}\n- VALIDATION samples:\t{}\n- TESTING samples:\t{}\n".format(len(train_set), len(valid_set),len(test_set)))


########################
### BUILD DATALOADER ###
########################
labels = train_set._get_labels()
print("Labels list ({}): {}".format(len(labels), labels))

# Computing the label weights to balance the dataloader
weights = train_set._get_label_weights()
print("The label weights are the followings.\n", weights)

# Building a weighted sampler
pin_memory = True if settings.training.device=="cuda" else False

train_sampler = torch.utils.data.sampler.WeightedRandomSampler(weights=weights, num_samples=len(train_set))
valid_sampler = torch.utils.data.sampler.WeightedRandomSampler(weights=weights, num_samples=len(valid_set))
test_sampler = torch.utils.data.sampler.WeightedRandomSampler(weights=weights, num_samples=len(test_set))

train_loader = torch.utils.data.DataLoader(dataset=train_set, batch_size=settings.training.batch_size, sampler=train_sampler, num_workers=settings.training.num_workers, collate_fn=_custom_collate_fn, pin_memory=pin_memory)
valid_loader = torch.utils.data.DataLoader(dataset=valid_set, batch_size=settings.training.batch_size, sampler=valid_sampler, num_workers=settings.training.num_workers, collate_fn=_custom_collate_fn, pin_memory=pin_memory)
test_loader = torch.utils.data.DataLoader(dataset=test_set, batch_size=settings.training.batch_size, sampler=test_sampler, num_workers=settings.training.num_workers, collate_fn=_custom_collate_fn, pin_memory=pin_memory)

for batch_ndx, sample in enumerate(train_loader):
    pass