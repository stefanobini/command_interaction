import os
import pandas
import sys
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
import argparse
from tqdm import tqdm

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
    print(Back.GREEN + "CUDA acceleration available on {} devices, you select {}".format(torch.cuda.device_count(), settings.training.devices))
# Set CUDA device
devices = ""
for device in settings.training.devices:
    devices += str(device) + ", "
devices = devices[:-2]
os.environ["CUDA_VISIBLE_DEVICES"] = devices
device = settings.training.devices[0]

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
    labels = labels_1
    task_n_labels = list((len(labels_1), len(labels_2)))
elif settings.task == "SCR":
    labels = labels_1
elif settings.task == "SI":
    labels = labels_2
else:
    sys.exit("The <{}> task is not allowed.".format(settings.task))


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
model = model.cuda(device=device)

#########################
#   k-Nearest Neighbor  #
#########################
# Set similarity metric
if settings.model.knn.similarity_fn == "cosine_similarity":
    similarity_fn = nn.CosineSimilarity(dim=settings.model.knn.dim, eps=settings.model.knn.eps)
# Initialize Nearest neighbor classifier
knn_classifier = NearestNeighbor(settings=settings, similarity_fn=similarity_fn)

accuracies = {"sampleXspeaker":list(), "accuracy":list()}
for n_samples_per_speaker in settings.dataset.knn.n_samples_per_speaker:
    # Load the dataloaders
    train_set = knnMiviaDataset(settings=settings, subset="training", n_samples_per_speaker=n_samples_per_speaker)
    test_set = knnMiviaDataset(settings=settings, subset="testing", n_samples_per_speaker=n_samples_per_speaker)

    # Initialiaze prototypes list
    #prototypes = np.zeros(shape=(len(train_set), settings.model.resnet8.out_channel)), np.zeros(shape=len(train_set))
    x_train = torch.zeros(size=(len(train_set), settings.model.resnet8.out_channel), device="cuda:{}".format(device))
    y_train = torch.zeros(len(train_set), device="cuda:{}".format(device))
    index = 0

    # Build prototypes with training samples
    train_iter = tqdm(train_set)
    for _, x, _, _, _, speaker, command in train_iter:
        # Predict the outputs
        if settings.task == "SCR_SI":
            predicted_command, speaker_embedding = model.get_embeddings(x=x.unsqueeze(dim=0).cuda(device=device))
        elif settings.task == "SI":
            speaker_embedding = model.get_embeddings(x=x.unsqueeze(dim=0).cuda(device=device))
        elif settings.task == "SCR":
            predicted_command = model(x=x)
        # Collect prototypes
        x_train[index] = speaker_embedding.squeeze()
        y_train[index] = speaker
        index += 1

        train_iter.set_description("Training phase with <{}> samples per speaker".format(n_samples_per_speaker))
    
    # Train k-Nearest Neighbor
    knn_classifier.train(X=x_train, y=y_train)            
    
    # Testing phase
    y_test = torch.zeros(len(test_set), device="cuda:{}".format(device))
    predictions = torch.zeros(len(test_set), device="cuda:{}".format(device))
    index = 0

    test_iter = tqdm(test_set)
    for _, x, _, _, _, speaker, command in test_iter:
        # Predict the outputs
        if settings.task == "SCR_SI":
            predicted_command, speaker_embedding = model.get_embeddings(x=x.unsqueeze(dim=0).cuda(device=device))
        elif settings.task == "SI":
            speaker_embedding = model.get_embeddings(x=x.unsqueeze(dim=0).cuda(device=device))
        elif settings.task == "SCR":
            predicted_command = model(x=x)
        predicted_speaker = knn_classifier.predict(X=speaker_embedding.squeeze())
        
        # Collect results
        y_test[index] = speaker
        predictions[index] = predicted_speaker
        index += 1

        test_iter.set_description("Testing phase with <{}> samples per speaker".format(n_samples_per_speaker))

    # Compute metrics
    accuracy = torchmetrics.functional.classification.accuracy(preds=predictions, target=y_test, task="multiclass", num_classes=task_n_labels[1], average="micro")
    accuracies["sampleXspeaker"].append(n_samples_per_speaker)
    accuracies["accuracy"].append(accuracy*100)
    print("ACCURACY with <{}> samples per speaker is: {:.2f} %".format(n_samples_per_speaker, accuracy*100))

# Save results
os.makedirs(results_path, exist_ok=True)
results_file = os.path.join(results_path, "srid_results.csv")

HEADINGS = ["sampleXspeaker", "accuracy"]
df = pandas.DataFrame(data=accuracies, columns=HEADINGS)
pandas.to_csv(path_or_buf=df, index=False)

# https://github.com/Kulbear/pytorch-the-hard-way/blob/master/Solutions/Nearest%20Neighbor.ipynb