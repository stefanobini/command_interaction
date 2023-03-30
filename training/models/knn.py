import sys
import os
from typing import List, Tuple, Dict
import numpy as np
from dotmap import DotMap
import torch


class NearestNeighbor(object):

    def __init__(self, settings:DotMap, similarity_fn) -> None:
        super().__init__()
        self.distance_fn = similarity_fn


    def train(self, X:torch.Tensor, y:torch.Tensor):
        """ X is N x D where each row is an example. Y is 1-dimension of size N """
        # the nearest neighbor classifier simply remembers all the training data
        self.Xtr = X
        self.ytr = y


    def predict(self, X:torch.Tensor) -> torch.Tensor:
        """ X is N x D where each row is an example we wish to predict label for """
        num_test = X.shape[0]
        # lets make sure that the output type matches the input type
        Ypred = torch.zeros(num_test, dtype=self.ytr.dtype)

        # loop over all test rows
        for i in range(num_test):
            # find the nearest training image to the i'th test image
            # using the L1 distance (sum of absolute value differences)
            distances = torch.sum(torch.abs(self.Xtr - X[i,:]), dim=1)
            # using the L2 distance (sum of absolute value differences)
            #distances = np.sqrt(np.sum(np.square(self.Xtr - X[i,:]), axis=1))
            min_index = torch.argmin(distances) # get the index with smallest distance
            Ypred[i] = self.ytr[min_index] # predict the label of the nearest example

        return Ypred