import os
import pandas as pd
from typing import List, Tuple, Dict
import colorama
colorama.init(autoreset=True)
from colorama import Back, Fore
import numpy as np
import matplotlib as mpl
import librosa

import torch
from torch.utils.data import Dataset
import torchaudio

from settings.conf_1 import settings


try:
    # If run from the parent folder
    from utils.preprocessing import plot_melspectrogram, plot_mfcc, Preprocessing, save_audio
except ModuleNotFoundError:
    # If run from the "utils" folder
    from preprocessing import plot_melspectrogram, plot_mfcc, Preprocessing, save_audio


it = 0
class MiviaDataset(Dataset):
    
    def __init__(self) -> Dataset:
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
        self.preprocessing = Preprocessing()
        self.dataset_path = settings.dataset.folder
        self.speech_annotations = None
        self.noise_annotations = None
    

    def __len__(self) -> int:
        return len(self.speech_annotations)


    def __getitem__(self, index) -> Tuple[torch.FloatTensor, int, str, str, str, int]:
        item = self.speech_annotations.iloc[index]
        rel_path = item.path
        full_path = os.path.join(self.dataset_path, rel_path)
        waveform, sample_rate = torchaudio.load(full_path)
        waveform = self.preprocessing.resample_audio(waveform=waveform, sample_rate=sample_rate)  # uniform sample rate
        waveform = torch.mean(input=waveform, dim=0, keepdim=True)  # reduce to one channel
        
        if settings.input.type == "mfcc":
            mfcc = self.preprocessing.get_mfcc(waveform)
            return mfcc, settings.input.sample_rate, item.type, item.subtype, item.speaker, int(item.label)
        elif settings.input.type == "melspectrogram":
            melspectrogram = self.preprocessing.get_melspectrogram(waveform)   # (channel, n_mels, time)
            return melspectrogram, sample_rate, item.type, item.subtype, item.speaker, int(item.label)

        return waveform, sample_rate, item.type, item.subtype, item.speaker, int(item.label)


    def _get_labels(self) -> List[int]:
        return list(self.speech_annotations.groupby("label").groups.keys())


    def _get_label_weights(self) -> List[torch.FloatTensor]:
        """ Calculate the percentage of how many samples there are for each class. The dataloader can be balanced through these weights.
        
        Returns
        -------
        torch.FloatTensor
            One-Dimensional FloatTensor of weights.
        """
        labels_group = self.speech_annotations.groupby("label")
        weights = torch.zeros(len(self._get_labels()))

        indexes = range(len(self._get_labels()))
        for index, label in zip(indexes, self._get_labels()):
            weights[index] = len(labels_group.get_group(name=label)) / len(self.speech_annotations)
        
        return weights


class TrainingMiviaDataset(MiviaDataset):

    def __init__(self) -> Dataset:
        super().__init__()
        
        self.dataset_path = os.path.join(settings.dataset.folder, "training")
        self.speech_annotations = pd.read_csv(settings.dataset.speech.training.annotations, sep=',')
        self.noise_annotations = pd.read_csv(settings.dataset.noise.training.annotations, sep=',')
        self.epoch = 0

    
    def __getitem__(self, index) -> Tuple[torch.FloatTensor, int, str, str, str, int]:
        speech_item = self.speech_annotations.iloc[index]
        rel_speech_path = speech_item.path
        speech_path = os.path.join(self.dataset_path, rel_speech_path)
        speech, speech_sample_rate = torchaudio.load(filepath=speech_path)
        #speech, speech_sample_rate = librosa.core.load(speech_path, sr=settings.input.sample_rate, mono=True)
        speech = self.preprocessing.resample_audio(waveform=speech, sample_rate=speech_sample_rate)  # uniform sample rate
        speech = torch.mean(input=speech, dim=0, keepdim=True)  # reduce to one channel

        # Remove silences
        '''
        no_silent_ranges = detect_nonsilent(audio_segment=speech)
        no_silent_indices = ranges2list(no_silent_ranges)
        no_silent_speech = torch.index_select(input=speech, dim=0, index=torch.tensor(no_silent_indices))
        print("Speech size: {}\tNo-silent speech size: {}\n".format(speech.size(), no_silent_speech.size()))
        #'''

        noise_index = index % len(self.noise_annotations)
        noise_item = self.noise_annotations.iloc[noise_index]
        rel_noise_path = noise_item.path
        noise_path = os.path.join(self.dataset_path, rel_noise_path)
        noise, noise_sample_rate = torchaudio.load(filepath=noise_path)
        # noise, noise_sample_rate = librosa.core.load(noise_path, sr=settings.input.sample_rate, mono=True)
        noise = self.preprocessing.resample_audio(waveform=noise, sample_rate=noise_sample_rate)     # uniform sample rate
        noise = torch.mean(input=noise, dim=0, keepdim=True)    # reduce to one channel
        
        snr = self.preprocessing.compute_snr(epoch=self.epoch)
        item = self.preprocessing.get_noisy_speech(speech=speech, noise=noise, snr_db=snr) if snr is not None else speech

        '''
        global it
        if it < 1:
            save_audio(wav_path="check_files/waveforms/{}_{}_{}".format(speech_item.label, snr, rel_speech_path.replace('/', '-')), waveform=item, sample_rate=settings.input.sample_rate)
        '''
        if settings.input.type == "waveform":
            return rel_speech_path, item, settings.input.sample_rate, speech_item.type, speech_item.subtype, speech_item.speaker, int(speech_item.label)
        elif settings.input.type == "mfcc":
            item = self.preprocessing.get_mfcc(item)
        elif settings.input.type == "melspectrogram":
            item = self.preprocessing.get_melspectrogram(item)   # (channel, n_mels, time)
        
        #########################################################################
        # ATTENZIONEEEEEEEE!!!!!!!!!!!!!!!! NON SO SE IL MULTIPLIER è 10. o 20. #
        #########################################################################
        if settings.input.spectrogram.type == "db":
            item = self.preprocessing.power_to_db_spectrogram(spectrogram=item)
        
        if settings.input.spectrogram.normalize:
            item = normalize_tensor(tensor=item)

        return rel_speech_path, item, settings.input.sample_rate, speech_item.type, speech_item.subtype, speech_item.speaker, int(speech_item.label)


    def set_epoch(self, epoch:int=0) -> None:
        self.epoch = epoch

    
    def _shuffle_noise_dataset(self) -> None:
        """ Shuffle noise annotation file. The method is useful to change the loading order among epochs. """
        self.noise_annotations =self.noise_annotations.sample(frac=1)


class ValidationMiviaDataset(MiviaDataset):

    def __init__(self) -> Dataset:
        super().__init__()

        self.dataset_path = os.path.join(settings.dataset.folder, "validation")
        self.speech_annotations = pd.read_csv(settings.dataset.speech.validation.annotations, sep=',')
    

    def __getitem__(self, index) -> Tuple[torch.FloatTensor, int, str, str, str, int, int]:
        pre_item = self.speech_annotations.iloc[index]
        rel_path = pre_item.path
        full_path = os.path.join(self.dataset_path, rel_path)
        waveform, sample_rate = torchaudio.load(filepath=full_path)
        # waveform, sample_rate = librosa.core.load(full_path, sr=settings.input.sample_rate, mono=True)
        waveform = self.preprocessing.resample_audio(waveform=waveform, sample_rate=sample_rate)  # uniform sample rate
        item = torch.mean(input=waveform, dim=0, keepdim=True)  # reduce to one channel
        
        if settings.input.type == "waveform":
            return rel_path, item, settings.input.sample_rate, pre_item.type, pre_item.subtype, pre_item.speaker, int(pre_item.label), pre_item.snr
        elif settings.input.type == "mfcc":
            item = self.preprocessing.get_mfcc(item)
        elif settings.input.type == "melspectrogram":
            item = self.preprocessing.get_melspectrogram(item)   # (channel, n_mels, time)
        
        #########################################################################
        # ATTENZIONEEEEEEEE!!!!!!!!!!!!!!!! NON SO SE IL MULTIPLIER è 10. o 20. #
        #########################################################################
        if settings.input.spectrogram.type == "db":
            item = self.preprocessing.power_to_db_spectrogram(spectrogram=item)
        
        if settings.input.spectrogram.normalize:
            item = normalize_tensor(tensor=item)

        return rel_path, item, settings.input.sample_rate, pre_item.type, pre_item.subtype, pre_item.speaker, int(pre_item.label), pre_item.snr


class TestingMiviaDataset(MiviaDataset):

    def __init__(self) -> Dataset:
        super().__init__()

        self.dataset_path = os.path.join(settings.dataset.folder, "testing")
        self.speech_annotations = pd.read_csv(settings.dataset.speech.testing.annotations, sep=',')

    
    def __getitem__(self, index) -> Tuple[torch.FloatTensor, int, str, str, str, int, int]:
        item = self.speech_annotations.iloc[index]
        rel_path = item.path
        full_path = os.path.join(self.dataset_path, rel_path)
        waveform, sample_rate = torchaudio.load(full_path)
        waveform = self.preprocessing.resample_audio(waveform=waveform, sample_rate=sample_rate)  # uniform sample rate
        waveform = torch.mean(input=waveform, dim=0, keepdim=True)  # reduce to one channel
        
        if settings.input.type == "mfcc":
            mfcc = self.preprocessing.get_mfcc(waveform)
            return rel_path, mfcc, sample_rate, item.type, item.subtype, item.speaker, int(item.label), int(item.snr)
        elif settings.input.type == "melspectrogram":
            melspectrogram = self.preprocessing.get_melspectrogram(waveform)   # (channel, n_mels, time)
            return rel_path, melspectrogram, sample_rate, item.type, item.subtype, item.speaker, int(item.label), int(item.snr)

        return rel_path, waveform, sample_rate, item.type, item.subtype, item.speaker, int(item.label), int(item.snr)
            

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


def normalize_tensor(tensor:torch.Tensor) -> torch.Tensor:
    mean = torch.mean(tensor)
    variance = torch.var(tensor) if torch.var(tensor)>1e-6 else 1e-6
    normalized_tensor = (tensor-mean) / variance
    return normalized_tensor


def pad_spectrograms(batch:torch.FloatTensor, max_lenght:int, padding_value:float=0.) -> torch.FloatTensor:
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
    resized_batch = torch.zeros(size=(len(batch), batch[0].size(0), batch[0].size(1), max_lenght))              # (B x C x F x T)
    # to optimize try to assign block per clock the sample
    for b in range(resized_batch.size(0)):
        #sample_with_silent_queue = torch.ones(size=(batch[0].size(0), batch[0].size(1), batch[b].size(2)+settings.input.spectrogram.padding.stride)) * settings.input.spectrogram.padding.value
        #sample_with_silent_queue[:, :, :batch[b].size(2)] = batch[b]                                             
        # # add a silence part after the sample to separate each repetition of the sample in the batch
        sample_with_silent_queue = torch.nn.ConstantPad2d((0,settings.input.spectrogram.padding.stride,0,0), value=settings.input.spectrogram.padding.value)(batch[b])  # i can pad noise instead a fixed value
        resized_batch[b, 0, :, :batch[b].size(2)] = batch[b]
        for f in range(resized_batch.size(2)):
            for t in range(batch[b].size(2), resized_batch.size(3)):
                resized_batch[b, 0, f, t] = sample_with_silent_queue[0, f, t%sample_with_silent_queue.size(2)]                                        # pad zero value replicating the spectrogram
    # Normalize batch
    '''
    batch_mean = torch.mean(resized_batch)
    batch_var = torch.var(resized_batch) if torch.var(resized_batch)>1e-6 else 1e-6
    resized_batch = (resized_batch-batch_mean) / batch_var
    #'''
    # batch = torch.nn.utils.rnn.pad_sequence(sequences=batch, batch_first=True, padding_value=padding_value)    # (B x L x C x T)
    # return batch.permute(0, 2, 3, 1)    # (B x C x F x T)
    return resized_batch


def _train_collate_fn(batch:List[torch.FloatTensor]) -> Tuple[torch.FloatTensor, torch.IntTensor]:
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
    #print(Back.YELLOW + "TRAIN COLLATE FN" + Back.RESET)
    tensors, targets = list(), list()
    #paths = list()
    max_lenght = 0
    for path, tensor, _, _, _, _, label in batch:
        #paths.append(path.replace('/', '-').replace(".wav", ''))
        '''
        if True in torch.isnan(tensor):
            print(Back.RED + "{} with size {} has {} 'nan' value".format(path, tensor.size(), torch.count_nonzero(torch.isnan(tensor))))
        if True in torch.isinf(tensor):
            print(Back.RED + "{} has {} 'inf' value".format(path, tensor.size(), torch.count_nonzero(torch.isinf(tensor))))
        #'''
        max_lenght = tensor.size(2) if tensor.size(2)>max_lenght else max_lenght
        # tensors += [tensor.permute(2, 0, 1)]    # before: (CxFxT), after: (TxCxF)
        tensors += [tensor]    # tensor size (CxFxT)
        targets += [label_to_index(label)]
    tensors = pad_spectrograms(tensors, max_lenght=max_lenght, padding_value=settings.input.spectrogram.padding_value)
    targets = torch.stack(targets)
    # approx_snr = approximate_snr(snr=snr, multiple=settings.noise.snr_step)
    
    """
    global it
    for s in range(0, 3):
        #plot_mfcc(path="check_files/{}_{}_lab{}".format(it, s, targets[s]), mfcc=tensors[s])
        plot_melspectrogram(path="check_files/collate_fn/{}_{}_lab{}".format(it, s, targets[s]), melspectrogram=tensors[s])
        #plot_mfcc(path="check_files/{}_{}_{}_lab{}".format(it, s, paths[s], targets[s]), mfcc=tensors[s])
    it += 1
    #"""

    return tensors, targets#, approx_snr


def _val_collate_fn(batch:List[torch.FloatTensor]) -> Tuple[torch.FloatTensor, torch.IntTensor]:
    """ Process the audio samples in batch to have the same duration. It is used for validation phase because it manages a CombinedLoader
    The data tuple has the form:
    (waveform, sample_rate, type, subtype, speaker, label, snr)
    
    Parameters
    ----------
    batch: Dict[torch.FloatTensor]
        List of tensor contained in the batch

    Returns
    -------
    Dict[int, torch.FloatTensor]
        Combined batch (dictionary of batches) with audio sample of the same lenght
    """

    #print(Back.BLUE + "VALIDATION COLLATE FN" + Back.RESET)
    tensors, targets, snrs = list(), list(), list()
    #paths = list()
    max_lenght = 0
    for path, tensor, _, _, _, _, label, snr in batch:
        '''
        paths.append(path.replace('/', '-').replace(".wav", ''))
        if True in torch.isnan(tensor):
            print(Back.YELLOW + "{} with size {} has {} 'nan' value".format(path, tensor.size(), torch.count_nonzero(torch.isnan(tensor))))
        if True in torch.isinf(tensor):
            print(Back.YELLOW + "{} has {} 'inf' value".format(path, tensor.size(), torch.count_nonzero(torch.isinf(tensor))))
        '''
        max_lenght = tensor.size(2) if tensor.size(2)>max_lenght else max_lenght
        # tensors += [tensor.permute(2, 0, 1)]    # before: (CxFxT), after: (TxCxF)
        tensors += [tensor]    # tensor size (CxFxT)
        targets += [label_to_index(label)]
        snrs += [torch.tensor(snr)]
    tensors = pad_spectrograms(tensors, max_lenght=max_lenght, padding_value=settings.input.spectrogram.padding_value)
    targets = torch.stack(targets)
    snrs = torch.stack(snrs)

    '''
    global it
    for s in range(0, 2):
        #plot_mfcc(path="test/{}_{}_lab{}_snr{}".format(it, s, targets[s], snrs[s]), mfcc=tensors[s])
        plot_mfcc(path="check_files/{}_{}_{}_lab{}_snr{}".format(it, s, paths[s], targets[s], snrs[s]), mfcc=tensors[s])
    it += 1
    #'''
    return tensors, targets, snrs


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


def test_dataset(samples:List[int]) -> None:
    for i in range(len(samples)):
        if settings.input.type == "waveform":
            waveform, sample_rate, type, subtype, speaker, label = train_set[samples[i]]
            val_waveform, val_sample_rate, val_type, val_subtype, val_speaker, val_label, val_snr = valid_set[samples[i]]

            # print(train_set._get_sample_from_type("reject"))
            torchaudio.save("check_files/waveform_train_{}.wav".format(samples[i]), waveform, sample_rate) 
            torchaudio.save("check_files/waveform_val_{}.wav".format(samples[i]), val_waveform, val_sample_rate)
        elif settings.input.type == "melspectrogram":
            speech_path, mel_spectrogram, sample_rate, type, subtype, speaker, label = train_set[samples[i]]
            speech_path, val_mel_spectrogram, val_sample_rate, val_type, val_subtype, val_speaker, val_label, val_snr = valid_set[samples[i]]
            plot_melspectrogram(path="check_files/melspectrogram_train_{}_".format(samples[i]), melspectrogram=mel_spectrogram[0])
            plot_melspectrogram(path="check_files/melspectrogram_val_{}_".format(samples[i]), melspectrogram=val_mel_spectrogram[0])
        elif settings.input.type == "mfcc":
            speech_path, mfcc, sample_rate, type, subtype, speaker, label = train_set[samples[i]]
            speech_path, val_mfcc, val_sample_rate, val_type, val_subtype, val_speaker, val_label, val_snr = valid_set[samples[i]]
            plot_mfcc(path="check_files/mfcc_train_{}_".format(samples[i]), mfcc=mfcc[0])
            plot_mfcc(path="check_files/mfcc_val_{}_".format(samples[i]), mfcc=val_mfcc[0])


def test_dataloader(samples:List[int]) -> None:

    if settings.input.type == "mfcc":
        train_path = "check_files/mfcc_train_"
        val_path = "check_files/mfcc_val_"
    elif settings.input.type == "melspectrogram":
        train_path = "check_files/melspectrogram_train_"
        val_path = "check_files/melspectrogram_val_"

    train, val = list(), list()
    for i in range(len(samples)):
        path, mel_spectrogram, sample_rate, type, subtype, speaker, label = train_set[samples[i]]
        mel_spectrogram=mel_spectrogram.permute(2, 0, 1)
        path, val_mel_spectrogram, val_sample_rate, val_type, val_subtype, val_speaker, val_label, val_snr = valid_set[samples[i]]
        val_mel_spectrogram=val_mel_spectrogram.permute(2, 0, 1)
        train.append(mel_spectrogram)
        val.append(val_mel_spectrogram)
    t = pad_spectrograms(train, padding_value=settings.input.spectrogram.padding_value)
    v = pad_spectrograms(val, padding_value=settings.input.spectrogram.padding_value)
    for i in range(len(samples)):
        plot_melspectrogram(path=train_path+str(samples[i]), melspectrogram=t[i][0])
        plot_melspectrogram(path=val_path+str(samples[i]), melspectrogram=v[i][0])


if __name__ == "__main__":

    train_set = TrainingMiviaDataset()
    valid_set = ValidationMiviaDataset()
    test_set = TestingMiviaDataset()

    print("The dataset is splitted in:\n- TRAIN samples:\t{}\n- VALID samples:\t{}\n- TEST samples:\t\t{}".format(len(train_set), len(valid_set),len(test_set)))
    # print(train_set._get_label_weights())

    samples = [0, 500, 1000]
    test_dataset(samples=samples)
    test_dataloader(samples=samples)

    # labels = train_set._get_labels()
    # print("Labels list ({}): {}".format(len(labels), labels))

    # print("melspectrogram shape: {}\nSample rate: {}\nType: {}\nSubtype: {}\nSpeaker: {}\nLabel: {}".format(melspectrogram.size(), sample_rate, type, subtype, speaker, label))

    # plot_melspectrogram(melspectrogram=melspectrogram[0])
    '''
    plot_waveform(waveform=waveform)
    plot_spectrogram(waveform=waveform)
    db_spectrogram = get_spectrogram(waveform=waveform)
    plot_db_spectrogram(pow_spectrogram=db_spectrogram[0])
    '''