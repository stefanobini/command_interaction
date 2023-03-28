import os
import sys
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
import argparse
import numpy as np
import torch
import torch.nn as nn
from torchsummary import summary

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
pin_memory = True if settings.training.device=="cuda" else False

# Setting seed for reproducibility
# pl.seed_everything(5138)


#########################
#      Results path     #
#########################
info_path = os.path.join(settings.logger.folder, settings.logger.name, settings.logger.version)
print(Back.CYAN + "The TensorBoard logggers will be saved in <{}>.".format(info_path))


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
    model = ResNet8_PL(settings=settings, num_labels=num_labels, loss_weights=None).cuda()  # Load model
    model.load_state_dict(params["state_dict"])
    print(Back.BLUE + "LOAD PRETRAINED MODEL: {}".format(settings.model.resnet8.pretrain_path))
elif settings.model.network == "HS":
    params = torch.load(settings.model.hard_sharing.pretrain_path, lambda s, l: s)
    task_n_labels = params["hyper_parameters"]["task_n_labels"]
    state_dict = params["state_dict"]
    # del params["state_dict"]["loss_fn.weight"]
    model = HardSharing_PL(settings=settings, task_n_labels=task_n_labels, task_loss_weights=np.array(object=(None, None))).cuda()
    model.load_state_dict(state_dict)
    print(Back.BLUE + "LOAD PRETRAINED MODEL: {}".format(settings.model.hard_sharing.pretrain_path))
elif settings.model.network == "SS":
    params = torch.load(settings.model.soft_sharing.pretrain_path, lambda s, l: s)
    task_n_labels = params["hyper_parameters"]["task_n_labels"]
    state_dict = params["state_dict"]
    # del params["state_dict"]["loss_fn.weight"]
    model = SoftSharing_PL(settings=settings, task_n_labels=task_n_labels, task_loss_weights=np.array(object=(None, None))).cuda()
    model.load_state_dict(state_dict)
    print(Back.BLUE + "LOAD PRETRAINED MODEL: {}".format(settings.model.soft_sharing.pretrain_path))

# Model summary
input_shape = (1, 40, 100)
summary(model, input_shape)

#########################
#   k-Nearest Neighbor  #
#########################
# Setting similarity metric
if settings.model.knn.similarity_fn == "cosine_similarity":
    similarity_fn = nn.CosineSimilarity(dim=settings.model.knn.dim, eps=settings.model.knn.eps)

for n_samples_per_speaker in settings.dataset.knn.n_samples_per_speaker:
    # Load the dataloaders
    train_set = knnMiviaDataset(settings=settings, subset="training", n_samples_per_speaker=n_samples_per_speaker)
    val_set = knnMiviaDataset(settings=settings, subset="testing", n_samples_per_speaker=n_samples_per_speaker)

    # Initialiaze prototypes list
    prototypes = np.zeros(size=(len(train_set), settings.model.resnet8.out_channel)), np.zeros(size=(train_set))
    index = 0

    # Training phase
    speaker_group = train_set._get_speaker_group()
    for speaker in speaker_group.groups:
        df_speaker = speaker_group.get_group(speaker)
        for idx in df_speaker.index:
            # Get input sample
            x_rel_path, x, x_sample_rate, x_type, x_subtype, x_speaker, x_command = train_set[idx]
            # Predict the outputs
            if settings.task == "SCR_SI":
                predicted_command, speaker_embedding = model.get_embeddings(x=x.unsqueeze(dim=0).cuda())
            elif settings.task == "SI":
                speaker_embedding = model.get_embeddings(x=x.unsqueeze(dim=0).cuda())
            elif settings.task == "SCR":
                predicted_command = model(x=x)
            prototypes[index] = speaker_embedding.squeeze().numpy(), x_speaker
            index += 1
    srid_classifier = NearestNeighbor(settings=settings, similarity_fn=None)


            
    
    # Testing phase


# https://github.com/Kulbear/pytorch-the-hard-way/blob/master/Solutions/Nearest%20Neighbor.ipynb