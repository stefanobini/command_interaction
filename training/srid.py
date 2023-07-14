import os
import pandas
import sys
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
import argparse
from tqdm import tqdm
from typing import Tuple
from dotmap import DotMap

import numpy as np
import torch
import torch.nn as nn
from torchsummary import summary
import torchmetrics

from utils.dataloaders import knnMiviaDataset
from models.resnet8 import ResNet8_PL
from models.mobilenetv2 import MobileNetV2_PL
from models.conformer import Conformer_PL
from models.hardsharing import HardSharing_PL
# from models.hardsharing_mobilenetv2 import HardSharing_PL
from models.softsharing import SoftSharing_PL
# from models.softsharing_mobilenetv2 import SoftSharing_PL
from models.knn import NearestNeighbor


def build_embeddings(settings:DotMap, set_name:str, subset:torch.Tensor, model:torch.nn.Module) -> Tuple[torch.Tensor, torch.Tensor]:
    # Initialiaze prototypes list
    #prototypes = np.zeros(shape=(len(train_set), settings.model.resnet8.out_channel)), np.zeros(shape=len(train_set))
    x_set = torch.zeros(size=(len(subset), settings.model.resnet8.out_channel), device=settings.training.device)
    y_set = torch.zeros(len(subset), device=settings.training.device)
    index = 0

    # Build prototypes with training samples
    set_iter = tqdm(subset)
    for _, x, _, _, _, speaker, command in set_iter:
        # Predict the outputs
        if settings.task == "SCR_SI":
            predicted_command, speaker_embedding = model.get_embeddings(x=x.unsqueeze(dim=0).cuda(device=settings.training.device))
        elif settings.task == "SI":
            speaker_embedding = model.get_embeddings(x=x.unsqueeze(dim=0).cuda(device=settings.training.device))
        elif settings.task == "SCR":
            predicted_command = model(x=x)
        # Collect prototypes
        x_set[index] = speaker_embedding.squeeze().detach()
        y_set[index] = speaker
        index += 1

        set_iter.set_description("Loading the speakers embedding of the {} set".format(set_name))

    return x_set, y_set


# Acquiring parameters
parser = argparse.ArgumentParser()
parser.add_argument("--configuration", type=str, dest="configuration", required=True, help="Configuration file (e.g., 'conf_1')")
args = parser.parse_args()
args.configuration = "settings.{}".format(args.configuration)
settings = getattr(__import__(args.configuration, fromlist=["settings"]), "settings")
print(Back.CYAN + "Loaded <{}> as configuration file.".format(settings.name))


#########################
#          CUDA         #
#########################
if torch.cuda.is_available():
    available_devices = [i for i in range(torch.cuda.device_count())]
    # os.environ["CUDA_VISIBLE_DEVICES"] = str(available_devices)
    print(Back.GREEN + "CUDA acceleration available on <{}> devices: <{}>".format(torch.cuda.device_count(), available_devices))
    print(Back.GREEN + "The current device is <{}>, you select device <{}>".format(torch.cuda.current_device(), settings.training.device))
    pin_memory = True if settings.training.device=="cuda" else False

# Setting seed for reproducibility
# pl.seed_everything(5138)


#########################
#      Results path     #
#########################
results_path = os.path.join(settings.logger.folder, settings.logger.name, settings.logger.version)
print(Back.CYAN + "The TensorBoard logggers will be saved in <{}>.".format(results_path))


#########################
#         Labels        #
#########################
# Load model to produce the feature vector for SrID
dataset = knnMiviaDataset(settings=settings)
labels_1 = dataset._get_commands()
labels_2 = dataset._get_speakers()

if settings.task == "SCR_SI":
    labels = labels_2
    task_n_labels = list((len(labels_1), len(labels_2)))
elif settings.task == "SCR":
    labels = labels_1
elif settings.task == "SI":
    labels = labels_2
else:
    sys.exit("The <{}> task is not allowed.".format(settings.task))
num_speakers = len(labels)
# print("Number of speakers: {}".format(num_speakers))

#########################
#         Model         #
#########################
assert settings.model.network in ["resnet8", "mobilenetv2", "HS", "SS"]
model = None
if settings.model.network == "resnet8":
    params = torch.load(settings.model.resnet8.pretrain_path, lambda s, l: s)
    num_labels = params["hyper_parameters"]["num_labels"]
    state_dict = params["state_dict"]
    del params["state_dict"]["loss_fn.weight"]
    model = ResNet8_PL(settings=settings, num_labels=num_labels, loss_weights=None)  # Load model
    model.load_state_dict(params["state_dict"])
    print(Back.BLUE + "LOAD PRETRAINED MODEL: {}".format(settings.model.resnet8.pretrain_path))
elif settings.model.network == "HS":
    params = torch.load(settings.model.hard_sharing.pretrain_path, lambda s, l: s)
    task_n_labels = params["hyper_parameters"]["task_n_labels"]
    state_dict = params["state_dict"]
    # del params["state_dict"]["loss_fn.weight"]
    model = HardSharing_PL(settings=settings, task_n_labels=task_n_labels, task_loss_weights=np.array(object=(None, None)))
    model.load_state_dict(state_dict)
    print(Back.BLUE + "LOAD PRETRAINED MODEL: {}".format(settings.model.hard_sharing.pretrain_path))
elif settings.model.network == "SS":
    params = torch.load(settings.model.soft_sharing.pretrain_path, lambda s, l: s)
    task_n_labels = params["hyper_parameters"]["task_n_labels"]
    state_dict = params["state_dict"]
    # del params["state_dict"]["loss_fn.weight"]
    model = SoftSharing_PL(settings=settings, task_n_labels=task_n_labels, task_loss_weights=np.array(object=(None, None)))
    model.load_state_dict(state_dict)
    print(Back.BLUE + "LOAD PRETRAINED MODEL: {}".format(settings.model.soft_sharing.pretrain_path))

# Model summary
input_shape = (1, 40, 100)
summary(model, input_shape, device="cpu")
model = model.cuda(device=settings.training.device)

#########################
#   k-Nearest Neighbor  #
#########################
# Set similarity metric
if settings.knn.metric == "similarity" and settings.knn.function == "cosine":
    metric_fn = nn.CosineSimilarity(dim=settings.knn.cosine_similarity.dim, eps=settings.knn.cosine_similarity.eps)
elif settings.knn.metric == "distance" and settings.knn.function == "euclidean":
    metric_fn = nn.PairwiseDistance(p=2, eps=settings.knn.cosine_similarity.eps)
# Initialize Nearest neighbor classifier
knn_classifier = NearestNeighbor(settings=settings, metric_fn=metric_fn)

accuracies = {"sampleXspeaker":list(), "accuracy":list()}
for n_samples_per_speaker in settings.knn.n_samples_per_speaker:
    print("################################")
    print("#   <{:2d}> samples per speaker   #".format(n_samples_per_speaker))
    print("################################")

    # Load the dataloaders
    train_set = knnMiviaDataset(settings=settings, subset="training", n_samples_per_speaker=n_samples_per_speaker)
    test_set = knnMiviaDataset(settings=settings, subset="testing", n_samples_per_speaker=n_samples_per_speaker)

    # Compute training embeddings
    x_train, y_train = build_embeddings(settings=settings, set_name="Training", subset=train_set, model=model)
    
    # Train k-Nearest Neighbor
    knn_classifier.train(X=x_train, y=y_train)
    if settings.knn.plotting:
        figure_path = os.path.join(results_path, "{:2d}_train_distribution.png".format(n_samples_per_speaker))
        knn_classifier.plot_train_samples(figure_path=figure_path)     # Plot training embedding represented with T-SNE
    
    # Compute embeddings
    x_test, y_test = build_embeddings(settings=settings, set_name="Testing", subset=test_set, model=model)
    
    # Test k-Nearest Neighbor
    predictions = knn_classifier.predict(X=x_test)

    # Compute metrics
    accuracy = torchmetrics.functional.classification.accuracy(preds=predictions, target=y_test, task="multiclass", num_classes=num_speakers, average="micro")
    accuracies["sampleXspeaker"].append(n_samples_per_speaker)
    accuracies["accuracy"].append("{:.2f}".format(accuracy.item()*100))
    print("Accuracy: {:.2f} %\n".format(accuracy*100))

# Save results
os.makedirs(results_path, exist_ok=True)
results_file = os.path.join(results_path, "srid_results.csv")

HEADINGS = ["sampleXspeaker", "accuracy"]
df = pandas.DataFrame(data=accuracies, columns=HEADINGS)
df.to_csv(path_or_buf=results_file, index=False)

# https://github.com/Kulbear/pytorch-the-hard-way/blob/master/Solutions/Nearest%20Neighbor.ipynb