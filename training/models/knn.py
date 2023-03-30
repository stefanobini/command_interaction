import sys
import os
import time
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict
from tqdm import tqdm
from dotmap import DotMap
import pandas as pd
import numpy as np
import torch
from sklearn.decomposition import PCA
from sklearn.manifold import TSNE
import seaborn as sns
from scipy import spatial


class NearestNeighbor(object):

    def __init__(self, settings:DotMap, metric_fn) -> None:
        super().__init__()
        self.settings = settings
        self.metric_fn = metric_fn


    def train(self, X:torch.Tensor, y:torch.Tensor):
        """ X is N x D where each row is an example. Y is 1-dimension of size N """
        # the nearest neighbor classifier simply remembers all the training data
        self.Xtr = X.cuda(self.settings.training.device)
        self.ytr = y.cuda(self.settings.training.device)
        self.num_labels = torch.unique(input=y).size(0)


    def predict(self, X:torch.Tensor) -> torch.Tensor:
        """ X is N x D where each row is an example we wish to predict label for """
        # lets make sure that the output type matches the input type
        Ypred = torch.zeros(X.size(0), dtype=self.ytr.dtype, device=self.settings.training.device)

        # loop over all test rows
        test_iter = tqdm(range(X.size(0)))
        for i in test_iter:
            # find the nearest training image to the i'th test image
            # using the L1 distance (sum of absolute value differences)
            #distances = torch.sum(torch.abs(self.Xtr - X[i,:]), dim=1)
            # using the L2 distance (sum of absolute value differences)
            #distances = torch.sqrt(torch.sum(torch.square(self.Xtr - X[i,:]), axis=1))
            
            metrics = torch.zeros(self.Xtr.size(0), dtype=self.ytr.dtype, device=self.settings.training.device)
            for j in range(self.Xtr.size(0)):
                metrics[j] = self.metric_fn(X[i], self.Xtr[j])

            # index = torch.argmax(metrics) # get the index with highest similarity
            # index = torch.argmin(metrics) # get the index with smallest distance
            index = torch.argmax(metrics) if self.settings.knn.metric == "similarity" else torch.argmin(metrics)
            Ypred[i] = self.ytr[index] # predict the label of the nearest example

            test_iter.set_description("Testing phase")

        return Ypred
    

    def plot_train_samples(self, figure_path):
        feat_cols = [str(i) for i in range(self.Xtr.size(1))]
        train_df = pd.DataFrame(data=self.Xtr.cpu().numpy(), columns=feat_cols)
        train_df['y'] = self.ytr.cpu().numpy()

        train_data = train_df[feat_cols].values

        pca = PCA(n_components=3)
        pca_result = pca.fit_transform(train_data)

        train_df["pca-one"] = pca_result[:, 0]
        train_df["pca-two"] = pca_result[:, 1]
        train_df["pca-three"] = pca_result[:, 2]

        # print('Explained variation per principal component: {}'.format(pca.explained_variance_ratio_))

        time_start = time.time()
        tsne = TSNE(n_components=2, verbose=0, perplexity=40, n_iter=300, metric="cosine")
        tsne_results = tsne.fit_transform(train_df)

        # print('t-SNE done! Time elapsed: {} seconds'.format(time.time()-time_start))

        train_df["tsne-2d-one"] = tsne_results[:, 0]
        train_df["tsne-2d-two"] = tsne_results[:, 1]

        plt.figure(figsize=(16,7))
        ax1 = plt.subplot(1, 2, 1)
        sns.scatterplot(
            x="pca-one", y="pca-two",
            hue="y",
            palette=sns.color_palette("hls", self.num_labels),
            data=train_df,
            #legend="full",
            alpha=0.3,
            ax=ax1
        )

        ax2 = plt.subplot(1, 2, 2)
        sns.scatterplot(
            x="tsne-2d-one", y="tsne-2d-two",
            hue="y",
            palette=sns.color_palette("hls", self.num_labels),
            data=train_df,
            #legend="full",
            alpha=0.3,
            ax=ax2
        )

        plt.savefig(fname=figure_path)