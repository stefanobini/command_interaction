import os
import random
import matplotlib.pyplot as plt
import librosa
import librosa.display
from typing import List, Tuple
import numpy as np
from dotmap import DotMap

import torch
import torchaudio
import torchaudio.transforms as T
import torchaudio.functional as F

import colorama
from colorama import Back, Fore
colorama.init(autoreset=True)


class Preprocessing():
    
    def __init__(self, settings:DotMap):
        """Class that preprocesses audio samples before sending them to the network.
    
        Parameters
        ----------
        settings: DotMap
            DotMap object containing the configuration.

        Methods
        -------
        resample_audio(self, waveform:torch.Tensor, sample_rate:int) -> torch.Tensor
            Resample an audio waverform given as input (waveform). The target sample rate has to be defined in configuration file, while the original one is passed as parameter (sample_rate)
        get_melspectrogram(self, waveform:torch.Tensor) -> torch.Tensor
            Compute Mel-Spectrogram of a waveform given as input (waveform). All the parameters (sample_rate, n_fft, win_length, hop_length, n_mels) are taken from the configuration file.
        get_mfcc(self, waveform:torch.Tensor) -> torch.Tensor
            Compute MFCC of a waveform given as input (waveform). All the parameters (sample_rate, n_fft, win_length, hop_length, n_mels, n_mfcc, dct_type, norm, log_mels) are taken from the configuration file.
        amplitude_to_db_spectrogram(self, spectrogram:torch.Tensor) -> torch.Tensor
            Convert power spectrogram in dB spectrogram.
        approximate_snr(self, snr:float, multiple:int) -> int
            Approximate SNR to have a pool of values grouped in multiples of "multiple" to allow for performance evaluation
        
        Returns
        -------
        Object
            Preprocessing object with all the methods useful for the audio preprocessing
        """
        super().__init__()
        self.settings = settings

    
    def resample_audio(self, waveform:torch.Tensor, sample_rate:int) -> torch.Tensor:
        """Resample an audio waveform.
        The target sample rate has to be defined in configuration file, while the original one is passed as parameter (sample_rate).
        
        Parameters
        ----------
        waveform: torch.Tensor
            The waveform of the audio
        sample_rate: int
            Sample rate of the input waveform

        Returns
        -------
        torch.Tensor
            Resampled waveform
        """
        resampler = T.Resample(orig_freq=sample_rate, new_freq=self.settings.input.sample_rate, dtype=waveform.dtype)
        return resampler(waveform)

    
    def get_melspectrogram(self, waveform:torch.Tensor) -> torch.Tensor:
        """Compute Mel-Spectrogram of a waveform given as input (waveform).
        The method use torchaudio library for the trasformation. All the parameters (sample_rate, n_fft, win_length, hop_length, n_mels) are taken from the configuration file.
        
        Parameters
        ----------
        waveform: torch.Tensor
            Waveform of an audio

        Returns
        -------
        torch.Tensor
            Mel spectrogram of the waveform
        """
        '''
        melspectrogram = librosa.feature.melspectrogram(y=np.array(waveform), sr=self.settings.input.sample_rate, S=None, n_fft=self.settings.input.spectrogram.n_fft, hop_length=self.settings.input.spectrogram.hop_length, win_length=self.settings.input.spectrogram.win_length, window='hann', center=True, pad_mode='constant', power=1.0)
        return torch.tensor(melspectrogram, dtype=torch.float)
        #'''
        transformation = torchaudio.transforms.MelSpectrogram(sample_rate=self.settings.input.sample_rate, n_fft=self.settings.input.spectrogram.n_fft, win_length=self.settings.input.spectrogram.win_length, hop_length=self.settings.input.spectrogram.hop_length, n_mels=self.settings.input.mel.n_mels)
        return transformation(waveform)
    

    def get_mfcc(self, waveform:torch.Tensor) -> torch.Tensor:
        """Compute MFCC of a waveform given as input (waveform).
        The method use torchaudio trasformation. All the parameters (sample_rate, n_fft, win_length, hop_length, n_mels, n_mfcc, dct_type, norm, log_mels) are taken from the configuration file.
        
        Parameters
        ----------
        waveform: torch.Tensor
            Waveform of an audio

        Returns
        -------
        torch.Tensor
            MFCC spectrogram of the waveform
        """
        mel_arg = {"n_fft":self.settings.input.spectrogram.n_fft, "win_length":self.settings.input.spectrogram.win_length, "hop_length":self.settings.input.spectrogram.hop_length, "n_mels":self.settings.input.mel.n_mels}
        transformation = torchaudio.transforms.MFCC(sample_rate=self.settings.input.sample_rate, n_mfcc=self.settings.input.mfcc.n_mfcc, dct_type=self.settings.input.mfcc.dct_type, norm=self.settings.input.mfcc.norm, log_mels=self.settings.input.mfcc.log_mels, melkwargs=mel_arg)
        return transformation(waveform)


    def amplitude_to_db_spectrogram(self, spectrogram:torch.Tensor) -> torch.Tensor:
        """Convert amplitude spectrogram in dB spectrogram.
        The method use torchaudio library.
        
        Parameters
        ----------
        spectrogram: torch.Tensor
            Spectrogram of the audio signal (Mel or MFCC)

        Returns
        -------
        torch.Tensor
            dB version of the input spectrogram
        """
        #### CHANGED HERE THE MULTIPLIER FROM 10. T0 20.
        # spectrogram_db = torchaudio.functional.amplitude_to_DB(x=spectrogram, multiplier=20., amin=1e-10, db_multiplier=np.log10(max(spectrogram.max(), 1e-10)).numpy(), top_db=80)
        spectrogram_db = torchaudio.functional.amplitude_to_DB(x=spectrogram, multiplier=20., amin=1e-10, db_multiplier=np.log10(max(spectrogram.max(), 1e-10)), top_db=80)
        return spectrogram_db


    def approximate_snr(self, snr:float, multiple:int) -> int:
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

    
    def compute_snr(self, epoch:int) -> float:
        """Calculate SNR to apply to specific audio sample given the epoch.
         The method uses the epoch to apply the desired Curriculum Learning (CL) strategy:
         - PEM, uniform between minimum SNR and maximum SNR (without CL),
         - UniCL_PEM_v1, uniform with increasing interval with 'b' fixed at the maximum SNR
         - UniCL_PEM_v2, uniform at interval set at one "step"
         - GaussCL_PEM_v1, Gaussian with fixed variance and decreasing mean
         - GaussCL_PEM_v2, Gaussian with variable variance and decreasing mean

        Parameters
        ----------
        epoch: int
            Current epoch

        Returns
        -------
        int
            SNR to apply to the audio sample
        """
        snr = None
        descent_epochs = self.settings.noise.curriculum_learning.epoch_saturation_time*self.settings.noise.descent_ratio
        if self.settings.noise.curriculum_learning.distribution == "PEM":
            snr = random.uniform(self.settings.noise.min_snr, self.settings.noise.max_snr)
        elif self.settings.noise.curriculum_learning.distribution == "UniCL_PEM_v1":
            a = epoch * (self.settings.noise.min_snr - self.settings.noise.max_snr) / descent_epochs + self.settings.noise.max_snr                           # modeled as a linear descending function plus a flat phase in the end
            b = self.settings.noise.max_snr
            snr = random.uniform(a=a, b=b)
        elif self.settings.noise.curriculum_learning.distribution == "Anti_UniCL_PEM_v1":
            a = self.settings.noise.min_snr
            b = epoch * (self.settings.noise.max_snr - self.settings.noise.min_snr) / descent_epochs
            snr = random.uniform(a=a, b=b)
        elif self.settings.noise.curriculum_learning.distribution == "UniCL_PEM_v2":
            a = epoch * (self.settings.noise.min_snr + self.settings.noise.curriculum_learning.uniform.step - self.settings.noise.max_snr) / descent_epochs + self.settings.noise.max_snr - self.settings.noise.curriculum_learning.uniform.step
            b = epoch * (self.settings.noise.min_snr + self.settings.noise.curriculum_learning.uniform.step - self.settings.noise.max_snr) / descent_epochs + self.settings.noise.max_snr
            snr = random.uniform(a=a, b=b)
        elif self.settings.noise.curriculum_learning.distribution == "Anti_UniCL_PEM_v2":
            a = epoch * (self.settings.noise.max_snr + self.settings.noise.curriculum_learning.uniform.step - self.settings.noise.min_snr) / descent_epochs
            b = a + self.settings.noise.curriculum_learning.uniform.step
            snr = random.uniform(a=a, b=b)
        elif self.settings.noise.curriculum_learning.distribution == "GaussCL_PEM_v1":
            # model mu as a combination of descent linear function plus a constant  ->  \_
            mu = epoch * (self.settings.noise.min_snr - self.settings.noise.max_snr) / descent_epochs + self.settings.noise.max_snr     # modeled as a linear descending function plus a flat phase in the end
            # model sigma as a triangular function                                  ->  \/
            s1 = epoch * (-self.settings.noise.curriculum_learning.gaussian.max_sigma) / self.settings.noise.curriculum_learning.epoch_saturation_time + self.settings.noise.curriculum_learning.gaussian.max_sigma      # modeled as a linear descending function
            s2 = epoch * self.settings.noise.curriculum_learning.gaussian.max_sigma / self.settings.noise.curriculum_learning.epoch_saturation_time                          # modeled as a linear ascending function
            sigma = max(s1, s2)
            snr = np.random.normal(loc=mu, scale=sigma)
        elif self.settings.noise.curriculum_learning.distribution == "Anti_GaussCL_PEM_v1":
            # model mu as a combination of descent linear function plus a constant  ->  \_
            mu = epoch * (self.settings.noise.max_snr - self.settings.noise.min_snr) / descent_epochs     # modeled as a linear ascending function plus a flat phase in the end
            # model sigma as a triangular function                                  ->  \/
            s1 = epoch * (-self.settings.noise.curriculum_learning.gaussian.max_sigma) / self.settings.noise.curriculum_learning.epoch_saturation_time + self.settings.noise.curriculum_learning.gaussian.max_sigma      # modeled as a linear descending function
            s2 = epoch * self.settings.noise.curriculum_learning.gaussian.max_sigma / self.settings.noise.curriculum_learning.epoch_saturation_time                          # modeled as a linear ascending function
            sigma = max(s1, s2)
            snr = np.random.normal(loc=mu, scale=sigma)
        elif self.settings.noise.curriculum_learning.distribution == "GaussCL_PEM_v2":
            mu = epoch * (self.settings.noise.min_snr - self.settings.noise.max_snr) / descent_epochs + self.settings.noise.max_snr     # modeled as a linear descending function plus a flat phase in the end
            sigma = self.settings.noise.curriculum_learning.gaussian.sigma
            snr = np.random.normal(loc=mu, scale=sigma)
        elif self.settings.noise.curriculum_learning.distribution == "Anti_GaussCL_PEM_v2":
            mu = epoch * (self.settings.noise.max_snr - self.settings.noise.min_snr) / descent_epochs     # modeled as a linear ascending function plus a flat phase in the end
            sigma = self.settings.noise.curriculum_learning.gaussian.sigma
            snr = np.random.normal(loc=mu, scale=sigma)
            
        # pre_snr = snr
        if snr > self.settings.noise.max_snr:
            snr = self.settings.noise.max_snr
            # snr = self.settings.noise.max_snr - snr + self.settings.noise.max_snr
            # raise Exception("Computed SNR ({}) greater than maximum SNR ({})".format(snr, self.settings.noise.max_snr))
        elif snr < self.settings.noise.min_snr:
            snr = self.settings.noise.min_snr
            # snr = self.settings.noise.min_snr - snr + self.settings.noise.min_snr
            # raise Exception("Computed SNR ({}) lower than minimum SNR ({})".format(snr, self.settings.noise.min_snr))
        
        # n_sample += 1
        # print("*****\nEpoch: {}/{}\tmu: {}\tsigma: {}\tsnr: {}({})[{}, {}]\n******\n".format(epoch, self.settings.noise.curriculum_learning.epoch_saturation_time, mu, sigma, snr, pre_snr, self.settings.noise.min_snr, self.settings.noise.max_snr))
        # print(Back.YELLOW + "*****\nEpoch: {}/{}\ta: {}\tb: {}\tsnr: {}({})[{}, {}]\n******\n".format(epoch, self.settings.noise.curriculum_learning.epoch_saturation_time, a, b, snr, pre_snr, self.settings.noise.min_snr, self.settings.noise.max_snr))
        return int(snr)


    def fix_noise_duration(self, speech:torch.Tensor, noise:torch.Tensor) -> torch.Tensor:
        """Adjust the noise duration to be equal to the speech. The noise is cutted if is too long and padded (by zeros) if it is too short.
        
        Parameters
        ----------
        speech: torch.Tensor
            Speech waveform tensor (1-D). Tensor whose duration should not be changed.
        noise: torch.Tensor
            Noise waveform tensor (1-D). Tensor whose duration should be changed.

        Returns
        -------
        torch.Tensor
            Noise waveform tensor (1-D) with the same duration of the speech give as input.
        """

        if noise.size(1) > speech.size(1):
            start = random.randint(a=0, b=noise.size(1)-speech.size(1))
            end = start + speech.size(1)
            noise = noise[0, start:end]
            noise = torch.unsqueeze(noise, dim=0)
        elif noise.size(1) < speech.size(1):
            start = random.randint(a=0, b=speech.size(1)-noise.size(1))
            end = start + noise.size(1)
            t = torch.zeros(size=speech.size())
            t[0, start:end] = noise
            noise = t
        return noise


    def get_noisy_speech(self, speech:torch.Tensor, noise:torch.Tensor, snr_db:float) -> torch.Tensor:
        """Apply a noise with specific SNR to a speech waveform.

        Parameters
        ----------
        speech: torch.Tensor
            Speech waveform
        noise: torch.Tensor
            Noise waveform
        snr_db: float
            SNR in dB applied to the speech sample

        Returns
        -------
        torch.Tensor
            Noisy speech tensor
        """
        # maybe also the noise application point in the speech should be variate randomly
        noise = self.fix_noise_duration(speech=speech, noise=noise)
        # Compute Root Mean Square of the speech and noise
        speech_rms = speech.norm(p=2)
        noise_rms = noise.norm(p=2)

        snr = 10**(snr_db/20)   # Compute SNR in amplitude domain. Before 10 (power domain)
        noise_gain = speech_rms / (snr * noise_rms)
        noise_gain = min(noise_gain, self.settings.input.noise.max_gain)
        return (noise_gain * noise + speech) / 2


def plot_melspectrogram(path:str, melspectrogram:torch.Tensor) -> None:
    """Produce the plot of a Mel-spectrogram tensor and save it to a specific path.
    
    Parameters
    ----------
    path: str
        Where to save the plotting
    melspectrogram: torch.Tensor
        Mel-spectrogram to plot
    """
    fig, ax = plt.subplots()
    #melspectrogram_db = librosa.power_to_db(melspectrogram, ref=np.max)
    melspectrogram_db = melspectrogram[0].numpy()
    img = librosa.display.specshow(melspectrogram_db, y_axis='mel', x_axis='time', ax=ax)
    ax.set(title='Mel spectrogram display')
    fig.colorbar(img, ax=ax, format="%+2.f dB")
    plt.savefig(path)

def plot_mfcc(path:str, mfcc:torch.Tensor) -> None:
    """Produce the plot of a MFCC tensor and save it to a specific path.
    
    Parameters
    ----------
    path: str
        Where to save the plotting
    melspectrogram: torch.Tensor
       MFCC to plot
    """
    fig, ax = plt.subplots()
    #mfcc_db = librosa.power_to_db(mfcc[0], ref=np.max)
    mfcc_db = mfcc[0].numpy()
    img = librosa.display.specshow(mfcc_db, y_axis='mel', x_axis='time', ax=ax)
    # plt.tight_layout()
    ax.set(title='MFCC')
    fig.colorbar(img, ax=ax, format="%+2.f dB")
    plt.savefig(path)



#########################################
#           USELESS FUNCTIONS           #
#########################################
def _plot(tensor:torch.Tensor, sample_rate:int, title:str) -> None:
    """Plot waveform or power spectrogram of an audio sample.

    Parameters
    ----------
    tensor: torch.Tensor
        Waveform or spectrogram of the audio sample
    sample_rate: int
        Sample rate of the sample
    title: str
        Title of the plotting
    """

    tensor.numpy

    num_channels, num_frames = tensor.shape
    time_axis = torch.arange(0, num_frames) / sample_rate
    
    figure, axes =plt.subplots(num_channels, 1)
    if (num_channels == 1):
        axes = [axes]
    for c in range(num_channels):
        if title == "Waveform":
            axes[c].plot(time_axis, tensor[c], linewidth=1)
            axes[c].grid(True)
        else:
            axes[c].specgram(tensor[c], Fs=sample_rate)
        if num_channels>1:
            axes[c].set_ylabel(f'Channel {c+1}')
    figure.suptitle(title)
    plt.show(block=False)
    path = os.path.join("figures", title+"_eaxample.png")
    plt.savefig(path)


def plot_waveform(waveform:torch.Tensor, sample_rate:int=16000) -> None:
    """Plot waveform pf an audio sample.

    Parameters
    ----------
    tensor: torch.Tensor
        Waveform of the audio sample
    sample_rate: int
        Sample rate of the sample
    """

    _plot(waveform, sample_rate, title="Waveform")


def plot_spectrogram(waveform:torch.Tensor, sample_rate:int=16000) -> None:
    """Plot power spectrogram of an audio sample.

    Parameters
    ----------
    tensor: torch.Tensor
        Waveform of the audio sample
    sample_rate: int
        Sample rate of the sample
    """

    _plot(waveform, sample_rate, title="Spectrogram")


def plot_db_spectrogram(pow_spectrogram) -> None:
    """Plot spectrogram in dB from a power spectrogram.

    Parameters
    ----------
    pow_spectrogram: torch.Tensor
        Power spectrogram of anudio sample
    """

    fig, axs = plt.subplots(1, 1)
    axs.set_title("T.Spectrogram")
    axs.set_ylabel("freq_bin")
    axs.set_xlabel("frame")
    db_spec = librosa.power_to_db(pow_spectrogram)
    im = axs.imshow(db_spec, origin="lower", aspect="auto")
    fig.colorbar(im, ax=axs)
    plt.show(block=False)
    path = os.path.join("figures", "Spectrogram_dB_example.png")
    plt.savefig(path)