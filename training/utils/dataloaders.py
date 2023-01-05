import os
import pandas as pd

from torch.utils.data import Dataset
import torchaudio

from preprocessing import plot_waveform, plot_spectrogram, plot_db_spectrogram, get_spectrogram


class MiviaDataset(Dataset):

    def __init__(self, annotations_file, dataset_path) -> Dataset:
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

        self.annotations = pd.read_csv(annotations_file, sep=',')
        self.dataset_path = dataset_path

    
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
    annotations_path = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1/annotations/ita/ita_dataset.csv"
    dataset_path = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1"

    dataset = MiviaDataset(annotations_path, dataset_path)

    print("The dataset contains {} samples.".format(len(dataset)))

    sample = 0
    waveform, label = dataset[sample]

    plot_waveform(waveform=waveform)
    plot_spectrogram(waveform=waveform)
    db_spectrogram = get_spectrogram(waveform=waveform)
    plot_db_spectrogram(pow_spectrogram=db_spectrogram[0])