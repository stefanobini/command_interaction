import os
import pandas as pd
from typing import List, Tuple, Dict
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
import numpy as np

import torch
from torch.utils.data import Dataset
import torchaudio

from settings.conf_1 import settings

try:
    # If run from the parent folder
    from utils.preprocessing import plot_waveform, plot_spectrogram, plot_db_spectrogram, plot_melspectrogram, get_spectrogram, Preprocessing
except ModuleNotFoundError:
    # If run from the "utils" folder
    from preprocessing import plot_waveform, plot_spectrogram, plot_db_spectrogram, plot_melspectrogram, get_spectrogram, Preprocessing


class MiviaDataset(Dataset):

    def __init__(self, subset:str, preprocessing:Preprocessing) -> Dataset:
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

        self.preprocessing = preprocessing
        self.speech_annotations = None
        self.noise_annotations = None
        if subset == "training":
            self.speech_annotations = pd.read_csv(settings.dataset.speech.training.annotations, sep=',')
            self.noise_annotations = pd.read_csv(settings.dataset.noise.training.annotations, sep=',')
        elif subset == "validation":
            self.speech_annotations = pd.read_csv(settings.dataset.speech.validation.annotations, sep=',')
            self.noise_annotations = pd.read_csv(settings.dataset.noise.validation.annotations, sep=',')
        elif subset == "testing":
            self.speech_annotations = pd.read_csv(settings.dataset.speech.testing.annotations, sep=',')
            self.noise_annotations = pd.read_csv(settings.dataset.noise.testing.annotations, sep=',')
        else:   # Load the entire dataset
            train_speech_set = pd.read_csv(settings.dataset.speech.training.annotations, sep=',')
            val_speech_set = pd.read_csv(settings.dataset.speech.validation.annotations, sep=',')
            test_speech_set = pd.read_csv(settings.dataset.speech.testing.annotations, sep=',')
            self.speech_annotations = pd.concat(objs=[train_speech_set, val_speech_set, test_speech_set])

            train_noise_set = pd.read_csv(settings.dataset.noise.training.annotations, sep=',')
            val_noise_set = pd.read_csv(settings.dataset.noise.validation.annotations, sep=',')
            test_noise_set = pd.read_csv(settings.dataset.noise.testing.annotations, sep=',')
            self.noise_annotations = pd.concat(objs=[train_noise_set, val_noise_set, test_noise_set])

        self.types_group = self.speech_annotations.groupby("type")
        self.subtypes_group = self.speech_annotations.groupby("subtype")
        self.speakers_group = self.speech_annotations.groupby("speaker")
        self.labels_group = self.speech_annotations.groupby("label")

        self.dataset_path = settings.dataset.folder
        self.epoch = 0
    

    def __len__(self) -> int:

        return len(self.speech_annotations)

    
    def increase_epoch(self, step:int=0) -> None:
        self.epoch += step

    
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

    
    def _shuffle_noise_dataset(self) -> None:
        """ Shuffle noise annotation file. The method is useful to change the loading order among epochs. """
        self.noise_annotations =self.noise_annotations.sample(frac=1)


    def __getitem__(self, index) -> Tuple[torch.FloatTensor, int, str, str, str, int, float]:
        speech_item = self.speech_annotations.iloc[index]
        rel_speech_path = speech_item.path
        speech_path = os.path.join(self.dataset_path, rel_speech_path)
        speech, speech_sample_rate = torchaudio.load(speech_path)
        speech = self.preprocessing.resample_audio(waveform=speech, sample_rate=speech_sample_rate)  # uniform sample rate
        speech = torch.mean(input=speech, dim=0, keepdim=True)  # reduce to one channel
        
        noise_index = index % len(self.noise_annotations)
        noise_item = self.noise_annotations.iloc[noise_index]
        rel_noise_path = noise_item.path
        noise_path = os.path.join(self.dataset_path, rel_noise_path)
        noise, noise_sample_rate = torchaudio.load(noise_path)
        noise = self.preprocessing.resample_audio(waveform=noise, sample_rate=noise_sample_rate)     # uniform sample rate
        noise = torch.mean(input=noise, dim=0, keepdim=True)    # reduce to one channel
        
        snr = self.preprocessing.compute_snr(epoch=self.epoch)
        waveform = self.preprocessing.get_noisy_speech(speech=speech, noise=noise, snr_db=snr) if snr is not None else speech
        
        if settings.input.type == "mel-spectrogram":
            melspectrogram = self.preprocessing.get_melspectrogram(waveform)   # (channel, n_mels, time)
            return melspectrogram, speech_sample_rate, speech_item.type, speech_item.subtype, speech_item.speaker, int(speech_item.label)

        return waveform, self.preprocessing.sample_rate, speech_item.type, speech_item.subtype, speech_item.speaker, int(speech_item.label)


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
            weights[index] = len(self.labels_group.get_group(name=label)) / len(self.speech_annotations)
        
        return weights


class TrainMiviaDataset(MiviaDataset):

    def __init__(self, subset: str, preprocessing: Preprocessing) -> Dataset:
        super().__init__(subset, preprocessing)

        self.noise_annotations = None



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


def pad_melspectrogram(batch:torch.FloatTensor) -> torch.FloatTensor:
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
    batch = torch.nn.utils.rnn.pad_sequence(sequences=batch, batch_first=True, padding_value=0.)    # (B x L x C x F)
    return batch.permute(0, 2, 3, 1)    # (B x C x F x L)


def _custom_collate_fn(batch:List[torch.FloatTensor]) -> Tuple[torch.FloatTensor, torch.IntTensor]:
    """ Process the audio samples in batch to have the same duration.
    The data tuple has the form:
    (waveform, sample_rate, type, subtype, speaker, label)
    
    Parameters
    ----------
    batch: List[torch.FloatTensor]
        List of tensor contained in the batch

    Returns
    -------
    Tuple[torch.FloatTensor, torch.IntTensor]
        Batch with audio sample of the same lenght
    """
    
    tensors, targets = list(), list()
    for tensor, _, _, _, _, label in batch:
        tensors += [tensor.permute(2, 0, 1)]    # (L x C x F)
        targets += [label_to_index(label)]
    tensors = pad_melspectrogram(tensors)
    targets = torch.stack(targets)
    # approx_snr = approximate_snr(snr=snr, multiple=settings.noise.snr_step)

    return tensors, targets#, approx_snr


def _val_collate_fn(batch:Dict[int, torch.FloatTensor]) -> Tuple[torch.FloatTensor, torch.IntTensor]:
    """ Process the audio samples in batch to have the same duration. It is used for validation phase because it manages a CombinedLoader
    The data tuple has the form:
    (waveform, sample_rate, type, subtype, speaker, label)
    
    Parameters
    ----------
    batch: Dict[torch.FloatTensor]
        List of tensor contained in the batch

    Returns
    -------
    Dict[int, torch.FloatTensor]
        Combined batch (dictionary of batches) with audio sample of the same lenght
    """
    tensors_dict = dict()
    targets_dict = dict()
    for snr in batch[0][0].keys():
        tensors, targets = list(), list()
        for tensor_dict, _, _, _, _, label in batch:
            tensors += tensor_dict[snr].permute(2, 0, 1)    # (L x C x F)
            targets += [label_to_index(label)]
        tensors = pad_melspectrogram(tensors)
        targets = torch.stack(targets)
        tensors_dict[snr] = tensors
        targets_dict[snr] = targets

    return tensors_dict, targets_dict


def approximate_snr(snr:float, multiple:int) -> int:
    """Approximate SNR to have a pool of values grouped in multiples of "multiple" to allow for performance evaluation
    
    Parameters
    ----------
    snr: float
        SNR applied to the speech sample
    multiple: int
        Multiple to create the pool
    
    Returns
    -------
    int
        Approximated SNR
    """
    return multiple * round(snr / multiple)


if __name__ == "__main__":

    train_set = MiviaDataset(subset="training", preprocessing=Preprocessing())
    valid_set = MiviaDataset(subset="validation", preprocessing=Preprocessing(snr=-10))
    test_set = MiviaDataset(subset="testing", preprocessing=Preprocessing(snr=40))

    print("The dataset is splitted in:\n- TRAIN samples:\t{}\n- VALID samples:\t{}\n- TEST samples:\t\t{}".format(len(train_set), len(valid_set),len(test_set)))
    # print(train_set._get_label_weights())

    sample = 3
    waveform, sample_rate, type, subtype, speaker, label = train_set[sample]

    val_waveform, val_sample_rate, _, _, _, _ = valid_set[sample]

    # print(train_set._get_sample_from_type("reject"))
    torchaudio.save("train_sample.wav", waveform, sample_rate) 
    torchaudio.save("val_sample.wav", val_waveform, val_sample_rate) 

    labels = train_set._get_labels()
    # print("Labels list ({}): {}".format(len(labels), labels))

    # print("Mel-Spectrogram shape: {}\nSample rate: {}\nType: {}\nSubtype: {}\nSpeaker: {}\nLabel: {}".format(melspectrogram.size(), sample_rate, type, subtype, speaker, label))

    # plot_melspectrogram(melspectrogram=melspectrogram[0])
    '''
    plot_waveform(waveform=waveform)
    plot_spectrogram(waveform=waveform)
    db_spectrogram = get_spectrogram(waveform=waveform)
    plot_db_spectrogram(pow_spectrogram=db_spectrogram[0])
    '''