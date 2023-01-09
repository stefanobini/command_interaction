import os
import pandas as pd

from torch.utils.data import Dataset
import torchaudio

from preprocessing import plot_waveform, plot_spectrogram, plot_db_spectrogram, get_spectrogram
from settings.conf_1 import settings


class MiviaDataset(Dataset):

    def __init__(self, set:str) -> Dataset:
        """Dataset class for the MiviaDataset inherited from torch.utils.data.Dataset class.

        Parameters
        ----------
        annotations_file: str
            Path of the annotation file
        dataset_path: str
            Path of the dataset folder

        Returns
        -------
        torch.utils.data.Dataset
            Dataset object
        """

        self.annotation = None
        if set == "train":
            self.annotations = pd.read_csv(settings.dataset.train.annotations.speech, sep=',')
        elif set == "valid":
            self.annotations = pd.read_csv(settings.dataset.valid.annotations.speech, sep=',')
        else:
            self.annotations = pd.read_csv(settings.dataset.test.annotations.speech, sep=',')

        self.dataset_path = settings.dataset.folder

    
    def __len__(self) -> int:

        return len(self.annotations)


    def __getitem__(self, index) -> tuple:
        item = self.annotations.iloc[index]
        path = item.path
        label = item.label

        item_path = os.path.join(self.dataset_path, path)
        waveform, sr = torchaudio.load(item_path)

        return waveform, label


if __name__ == "__main__":

    train_set = MiviaDataset(set="train")
    valid_set = MiviaDataset(set="valid")
    test_set = MiviaDataset(set="test")

    print("The dataset is splitted in:\n- TRAIN samples:\t{}\n- VALID samples:\t{}\n- TEST samples:\t\t{}".format(len(train_set), len(valid_set),len(test_set)))

    sample = 0
    waveform, label = train_set[sample]

    plot_waveform(waveform=waveform)
    plot_spectrogram(waveform=waveform)
    db_spectrogram = get_spectrogram(waveform=waveform)
    plot_db_spectrogram(pow_spectrogram=db_spectrogram[0])