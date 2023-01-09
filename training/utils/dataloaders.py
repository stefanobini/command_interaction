import os
import pandas as pd
from typing import List, Tuple
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore

import torch
from torch.utils.data import Dataset
import torchaudio
from torchaudio.transforms import MelSpectrogram

from settings.conf_1 import settings

try:
    # If run from the parent folder
    from utils.preprocessing import plot_waveform, plot_spectrogram, plot_db_spectrogram, get_spectrogram
except ModuleNotFoundError:
    # If run from the "utils" folder
    from preprocessing import plot_waveform, plot_spectrogram, plot_db_spectrogram, get_spectrogram


class MiviaDataset(Dataset):

    def __init__(self, subset:str) -> Dataset:
        """Dataset class for the MiviaDataset inherited from torch.utils.data.Dataset class. The constructor requires only the name of the partition to load. The path of the dataset and the annotation file are taken in the configuration file.

        Parameters
        ----------
        subset: str
            Partition of the dataset between {None, "training", "validation", "testing"}. By default, it is None, in this case the whole dataset is loaded

        Returns
        -------
        torch.utils.data.Dataset
            Dataset object
        """

        assert subset is None or subset in ["training", "validation", "testing"], (
            "When `subset` not None, it must take a value from "
            + "{None, 'training', 'validation', 'testing'}."
        )

        self.annotations = None
        if subset == "training":
            self.annotations = pd.read_csv(settings.dataset.training.annotations.speech, sep=',')
        elif subset == "validation":
            self.annotations = pd.read_csv(settings.dataset.validation.annotations.speech, sep=',')
        elif subset == "testing":
            self.annotations = pd.read_csv(settings.dataset.testing.annotations.speech, sep=',')
        else:
            train_set = pd.read_csv(settings.dataset.training.annotations.speech, sep=',')
            valid_set = pd.read_csv(settings.dataset.validation.annotations.speech, sep=',')
            test_set = pd.read_csv(settings.dataset.testing.annotations.speech, sep=',')
            self.annotations = pd.concat(objs=[train_set, valid_set, test_set])

        self.types_group = self.annotations.groupby("type")
        self.subtypes_group = self.annotations.groupby("subtype")
        self.speakers_group = self.annotations.groupby("speaker")
        self.labels_group = self.annotations.groupby("label")

        self.dataset_path = settings.dataset.folder
    

    def __len__(self) -> int:

        return len(self.annotations)

    
    def _get_types(self) -> List[str]:
        return list(self.types_group.groups.keys())
    
    def _get_subtypes(self) -> List[str]:
        return list(self.subtypes_group.groups.keys())
    
    def _get_speakers(self) -> List[str]:
        return list(self.speakers_group.groups.keys())
    
    def _get_labels(self) -> List[int]:
        return list(self.labels_group.groups.keys())

    
    def _get_sample_from_type(self, name:str) -> pd.core.frame.DataFrame:
        assert name in self._get_types(), ("The dataset does not contain type: ", name)
        return self.types_group.get_group(name)

    def _get_sample_from_subtype(self, name:str) -> pd.core.frame.DataFrame:
        assert name in self._get_subtypes(), ("The dataset does not contain subtype: ", name)
        return self.subtypes_group.get_group(name)
    
    def _get_sample_from_speaker(self, name:str) -> pd.core.frame.DataFrame:
        assert name in self._get_speakers(), ("The dataset does not contain speaker: ", name)
        return self.speakers_group.get_group(name)

    def _get_sample_from_labels(self, name:str) -> pd.core.frame.DataFrame:
        assert name in self._get_labels(), ("The dataset does not contain label: ", name)
        return self.labels_group.get_group(name)


    def __getitem__(self, index) -> Tuple[torch.FloatTensor, int, str, str, str, int]:
        item = self.annotations.iloc[index]
        path = item.path
        label = int(item.label)

        item_path = os.path.join(self.dataset_path, path)
        waveform, sample_rate = torchaudio.load(item_path)
        compute_melspectrogram = MelSpectrogram(sample_rate=settings.input.sample_rate, n_fft=settings.input.n_ftt, win_length=settings.input.win_lenght, hop_length=settings.input.hop_lenght, n_mels=settings.input.n_mels)
        melspectrogram = compute_melspectrogram(waveform)

        return waveform, sample_rate, item.type, item.subtype, item.speaker, label


    def _get_label_weights(self) -> List[torch.FloatTensor]:
        """ Calculate the percentage of how many samples there are for each class. The dataloader can be balanced through these weights.
        
        Returns
        -------
        torch.FloatTensor
            One-Dimensional FloatTensor of weights.
        """
        weights = torch.zeros(len(self._get_labels()))

        indexes = range(len(self._get_labels()))
        for index, label in zip(indexes, self._get_labels()):
            weights[index] = len(self.labels_group.get_group(name=label)) / len(self.annotations)
        
        return weights


#########################
### UTILITY FUNCTIONS ###
#########################
def label_to_index(label:int) -> torch.IntTensor:
    """ Convert label in Integer Tensor.
    
    Parameters
    ----------
    label: int
        Label of audio sample

    Returns
    -------
    torch.IntTensor
        Label of audiuo sample as Integer tensor
    """
    return torch.tensor(label)


def index_to_label(index:torch.IntTensor) -> int:
    """ Convert Integer Tensor in label.
    
    Parameters
    ----------
    index: torch.IntTensor
        Label of audiuo sample as Integer tensor

    Returns
    -------
    int
        Label of audio sample
    """
    return torch.IntTensor(label)


def pad_sequence(batch:torch.FloatTensor) -> torch.FloatTensor:
    """ Make all tensor in a batch the same lenght by padding with zeros.
    
    Parameters
    ----------
    batch: torch.FloatTensor
        Batch tensor with shape B x *

    Returns
    -------
    torch.FloatTensor
        Batch with audio sample of the same lenght
    """
    batch = [item.t() for item in batch]
    batch = torch.nn.utils.rnn.pad_sequence(sequences=batch, batch_first=True, padding_value=0.)
    return batch.permute(0, 2, 1)


def _custom_collate_fn(batch:torch.FloatTensor) -> Tuple[torch.FloatTensor, torch.IntTensor]:
    """ Process the audio samples in batch to have the same duration.
    The data tuple has the form:
    (waveform, sample_rate, type, subtype, speaker, label)
    
    Parameters
    ----------
    batch: torch.FloatTensor
        Batch tensor

    Returns
    -------
    Tuple[torch.FloatTensor, torch.IntTensor]
        Batch with audio sample of the same lenght
    """
    
    tensors, targets = list(), list()
    for waveform, _, _, _, _, label in batch:
        tensors += [waveform]
        targets += [label_to_index(label)]

    tensors = pad_sequence(tensors)
    targets = torch.stack(targets)

    return tensors, targets


if __name__ == "__main__":

    train_set = MiviaDataset(subset="training")
    valid_set = MiviaDataset(subset="validation")
    test_set = MiviaDataset(subset="testing")

    print("The dataset is splitted in:\n- TRAIN samples:\t{}\n- VALID samples:\t{}\n- TEST samples:\t\t{}".format(len(train_set), len(valid_set),len(test_set)))
    print(train_set._get_label_weights())

    sample = 0
    waveform, sample_rate, type, subtype, speaker, label = train_set[sample]

    # print(train_set._get_sample_from_type("reject"))

    labels = train_set._get_labels()
    print("Labels list ({}): {}".format(len(labels), labels))

    print("Sample rate: {}\nType: {}\nSubtype: {}\nSpeaker: {}\nLabel: {}".format(sample_rate, type, subtype, speaker, label))

    plot_waveform(waveform=waveform)
    plot_spectrogram(waveform=waveform)
    db_spectrogram = get_spectrogram(waveform=waveform)
    plot_db_spectrogram(pow_spectrogram=db_spectrogram[0])