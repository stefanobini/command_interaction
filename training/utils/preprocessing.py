import os
import shutil
import random
import matplotlib.pyplot as plt
import math
import librosa
import librosa.display
from typing import List, Tuple
import numpy as np

import torch
import torchaudio
import torchaudio.transforms as T
import torchaudio.functional as F

from settings.conf_1 import settings


class Preprocessing():
    def __init__(self):
        self.sample_rate = settings.input.sample_rate
        self.n_fft = settings.input.n_fft
        self.win_lenght = settings.input.win_lenght
        self.hop_lenght = settings.input.hop_lenght
        self.n_mels = settings.input.n_mels
        self.max_epochs = settings.training.max_epochs
        self.min_snr = settings.noise.min_snr
        self.max_snr = settings.noise.max_snr
        self.descent_ratio = settings.noise.descent_ratio
        self.distribution = settings.noise.curriculum_learning.distribution
        self.min_sigma = settings.noise.curriculum_learning.min_sigma
        self.max_sigma = settings.noise.curriculum_learning.max_sigma

    
    def resample_audio(self, waveform:torch.FloatTensor, sample_rate:int) -> torch.FloatTensor:
        """Resample a waveform of an audio.
        
        Parameters
        ----------
        waveform : torch.FloatTensor
            The waveform of the audio
        sample_rate: int
            Sample rate of the sample
        resample_rate: str
            New sample rate
        dtype: torch.TensorType
            Tensor type of the waveform

        Returns
        -------
        torch.FloatTensor
            Resampled waveform
        """
        resampler = T.Resample(orig_freq=sample_rate, new_freq=self.sample_rate, dtype=waveform.dtype)
        
        # Lowpass filter width: larger lowpass_filter_width -> more precise filter, but more computationally expensive
        # Rolloff: lower rolloff reduces the amount of aliasing, but it will also reduce some of the higher frequencies
        # Window function
        return resampler(waveform)

    
    def get_melspectrogram(self, waveform:torch.FloatTensor) -> torch.FloatTensor:
        transformation = torchaudio.transforms.MelSpectrogram(sample_rate=self.sample_rate, n_fft=self.n_fft, win_length=self.win_lenght, hop_length=self.hop_lenght, n_mels=self.n_mels)

        return transformation(waveform)

    
    def compute_snr(self, epoch:int) -> float:        
        descent_epochs = epoch*self.descent_ratio
        if self.distribution == "uniform":
            snr = random.uniform(self.min_snr, self.max_snr)
        elif self.distribution == "dynamic_uniform":
            ### CURRICULUM LEARNING ###
            a = epoch * (self.min_snr - self.max_snr) / descent_epochs + self.max_snr                           # modeled as a linear descending function plus a flat phase in the end
            # b = epoch * (self.min_snr - self.max_snr) / self.descent_epochs + self.max_snr + self.ab_uniform_step    # modeled as a linear descending function plus a flat phase in the end
            b = self.max_snr
            if b > self.max_snr:
                b = self.max_snr
            snr = random.uniform(a=a, b=b)
        elif self.distribution == "dynamic_normal":
            ### CURRICULUM LEARNING ###
            # model mu as a combination of descent linear function plus a constant  ->  \_
            mu = epoch * (self.min_snr - self.max_snr) / descent_epochs + self.max_snr     # modeled as a linear descending function plus a flat phase in the end
            # model sigma as a triangular function                                  ->  /\
            s1 = epoch * (-self.max_sigma) / self.max_epochs + self.max_sigma + self.min_sigma      # modeled as a linear descending function
            s2 = epoch * self.max_sigma / self.max_epochs + self.min_sigma                          # modeled as a linear ascending function
            sigma = min(s1, s2)   
            snr = np.random.normal(loc=mu, scale=sigma)
            if (sigma < self.min_sigma or sigma > self.max_sigma) and self.max_sigma != 0:
                raise Exception("Error on sigma computation: {} [{}, {}]".format(sigma, self.min_sigma, self.max_sigma))
            
        # pre_snr = snr
        if snr > self.max_snr:
            snr = self.max_snr
        elif snr < self.min_snr:
            snr = self.min_snr 
        
        # n_sample += 1
        # print("*****\nEpoch: {}/{}\tmu: {}\tsigma: {}\tsnr: {}({})[{}, {}]\n******\n".format(epoch, self.max_epochs, mu, sigma, snr, pre_snr, self.min_snr, self.max_snr))
        # print("*****\nEpoch: {}/{}\ta: {}\tb: {}\tsnr: {}({})[{}, {}]\n******\n".format(epoch, self.max_epochs, a, b, snr, pre_snr, self.min_snr, self.max_snr))
        return snr


    def fix_noise_duration(self, speech:torch.FloatTensor, noise:torch.FloatTensor) -> torch.FloatTensor:
        """Adjust the noise duration to be equal to the speech. The noise is cutted if is too long and padded (by zeros) if it is too short.
        
        Parameters
        ----------
        speech: torch.FloatTensor
            Speech waveform tensor (1-D). Tensor whose duration should not be changed.
        noise: torch.FloatTensor
            Noise waveform tensor (1-D). Tensor whose duration should be changed.

        Returns
        -------
        torch.FloatTensor
            Noise waveform tensor (1-D) with the same duration of the speech give as input.
        """
        if noise.size(1) > speech.size(1):
            start = random.randint(a=0, b=noise.size(1)-speech.size(1))
            end = start + speech.size(1)
            noise = noise[0, start:end]
        elif noise.size(1) < speech.size(1):
            start = random.randint(a=0, b=speech.size(1)-noise.size(1))
            end = start + noise.size(1)
            t = torch.zeros(size=speech.size())
            t[0, start:end] = noise
            noise = t
        return noise        


    def get_noisy_speech(self, speech:torch.FloatTensor, noise:torch.FloatTensor, snr_db:float) -> torch.FloatTensor:
        """Apply a noise with specific SNR to a speech waveform.

        Parameters
        ----------
        speech: torch.FloatTensor
            Speech waveform
        noise: torch.FloatTensor
            Noise waveform
        snr: float
            SNR in dB applied to the speech sample

        Returns
        -------
        torch.FloatTensor
            Noisy speech
        """
        
        noise = self.fix_noise_duration(speech=speech, noise=noise)

        speech_power = speech.norm(p=2)
        noise_power = noise.norm(p=2)
        
        snr = math.exp(snr_db / 10)
        scale = speech_power / (snr * noise_power)
        return (scale * noise + speech) / 2

'''
    data_rms = data.rms_db
    noise_gain_db = min(data_rms - noise.rms_db - snr_db, self._max_gain_db)
    noise.gain_db(noise_gain_db)

    def rms_db(self):
        mean_square = np.mean(self._samples ** 2)
        return 10 * np.log10(mean_square)

    def gain_db(self, gain):
        self._samples *= 10.0 ** (gain / 20.0)
'''


def audio_info(wav_path:str):
    """Gets information about an audio sample.

    Parameters
    ----------
    wav_path : str
        Path of the audio file (in "wav" format)

    Returns
    -------
    torchaudio.AudioMetaData
        AudioMetaData object with information about audio sample
    """

    return torchaudio.info(wav_path)


def load_audio(wav_path:str) -> Tuple[torch.FloatTensor, int]:
    """Load an audio file.

    Parameters
    ----------
    wav_path : str
        Path of the audio file (in "wav" format)

    Returns
    -------
    torch.FloatTensor
        Pytorch tensor normalized within [-1.0, 1.0] representing the waveform. The shape is (n_channel, n_frame)
    int
        Number of channels
    """

    return torchaudio.load(wav_path)


def save_audio(wav_path:str, waveform:torch.FloatTensor, sample_rate:int, encoding="PCM_S", bits_per_sample:int=16, format:str="wav") -> None:
    """Save an waveform in a audio file.

    Parameters
    ----------
    wav_path : str
        Path of the audio file (in "wav" format)
    waveform: torch.FloatTensor
        Float pytorch tensor representing the waveform of the audio
    sample_rate: int
        Sample rate of the audio
    encoding: str
        By default is "PCM-S"
    bits_per_sample: int
        By default is 16 
    format: str
        By default is "wav"

    """

    torchaudio.save(wav_path, waveform, sample_rate, encoding=encoding, bits_per_sample=bits_per_sample, format=format)


def _plot(tensor:torch.FloatTensor, sample_rate:int, title:str) -> None:
    """Plot waveform or power spectrogram of an audio sample.

    Parameters
    ----------
    tensor: torch.FloatTensor
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


def plot_waveform(waveform:torch.FloatTensor, sample_rate:int=16000) -> None:
    """Plot waveform pf an audio sample.

    Parameters
    ----------
    tensor: torch.FloatTensor
        Waveform of the audio sample
    sample_rate: int
        Sample rate of the sample
    """

    _plot(waveform, sample_rate, title="Waveform")


def plot_spectrogram(waveform:torch.FloatTensor, sample_rate:int=16000) -> None:
    """Plot power spectrogram of an audio sample.

    Parameters
    ----------
    tensor: torch.FloatTensor
        Waveform of the audio sample
    sample_rate: int
        Sample rate of the sample
    """

    _plot(waveform, sample_rate, title="Spectrogram")


def plot_db_spectrogram(pow_spectrogram) -> None:
    """Plot spectrogram in dB from a power spectrogram.

    Parameters
    ----------
    pow_spectrogram: torch.FloatTensor
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


def plot_melspectrogram(path:str, melspectrogram:torch.FloatTensor) -> None:
    fig, ax = plt.subplots()
    melspectrogram_db = librosa.power_to_db(melspectrogram, ref=np.max)
    img = librosa.display.specshow(melspectrogram_db, y_axis='mel', x_axis='time', ax=ax)
    ax.set(title='Mel spectrogram display')
    fig.colorbar(img, ax=ax, format="%+2.f dB")
    plt.savefig(path)


# DATA AUGMENTATION
## Applying effects and filtering
### Apply effect to file
def get_sample(path:str, effects:list) -> Tuple[torch.FloatTensor, int]:
    """Load an audio file and apply a list of effects.
    To list the available effects use: torchaudio.sox_effects.effect_names()

    Parameters
    ----------
    path: str
        Path of the audio file (in "wav" format)
    effects: list
        List of the effects to apply, each element (effect) contain a list of strings composed by the name of the effectv and the required parameters
    resample: int
        Sample rate to apply to the modified audio

    Returns
    -------
    torch.FloatTensor
        Modified waveform
    int
        Sample rate
    """
    
    return torchaudio.sox_effects.apply_effects_file(path=path, effects=effects)


### Apply effect to tensor
def apply_effects(waveform:torch.FloatTensor, effects:list, resample:int=16000) -> Tuple[torch.FloatTensor, int]:
    """Apply a list of effects to a waveform of an audio.
    To list the available effects use: torchaudio.sox_effects.effect_names()

    Parameters
    ----------
    waveform: torch.FloatTensor
        Waveform of the audio
    effects: list
        List of the effects to apply, each element (effect) contain a list of strings composed by the name of the effectv and the required parameters
    resample: int
        Sample rate to apply to the modified audio

    Returns
    -------
    torch.FloatTensor
        Modified waveform
    int
        Sample rate
    """
    
    return torchaudio.sox_effects.apply_effects_tensor(tensor=waveform, sample_rate=resample, effects=effects)


def get_noisy_speeches(speech:torch.FloatTensor, noise:torch.FloatTensor, speech_sample_rate:int=16000, noise_sample_rate:int=16000, SNRs:list=list()) -> List[torch.FloatTensor]:
    """Apply a noise with different SNRs to a speech waveform.

    Parameters
    ----------
    speech: torch.FloatTensor
        Speech waveform
    noise: torch.FloatTensor
        Noise waveform
    speech_sample_rate: int
        Sample rate of speech waveform
    noise_sample_rate: int
        Sample rate to apply to the noise waveform
    SNRs: list
        List of the SNRs (in dB) to apply on the speech waveform

    Returns
    -------
    list(torch.FloatTensor)
        List of the noisy speeches.
    """
    
    noisy_speeches = list()    
    for snr_db in SNRs:
        noisy_speech = get_noisy_speech(speech=speech, noise=noise, speech_sample_rate=speech_sample_rate, noise_sample_rate=noise_sample_rate, snr_db=snr_db)
        noisy_speeches.append(noisy_speech)
    
    return noisy_speeches


## Applying codec to Tensor object
def apply_codec(waveform:torch.FloatTensor, sample_rate:int, config:dict) -> torch.FloatTensor:
    """Apply a list of effects to a waveform of an audio.
    To list the available effects use: torchaudio.sox_effects.effect_names()

    Parameters
    ----------
    waveform: torch.FloatTensor
        Waveform of the audio
    sample_rate: int
        Sample rate of the audio waveform
    config: dict
        Dictionary containing the parameters required by the codec

    Returns
    -------
    torch.FloatTensor
        Waveform with new codec
    """
    
    return F.apply_codec(waveform=waveform, sample_rate=sample_rate, **config)


def get_spectrogram(self, waveform:torch.FloatTensor) -> torch.FloatTensor:
    """Get spectrogram in dB from a waveform.

    Parameters
    ----------
    waveform: torch.FloatTensor
        Waveform of the audio
    n_ftt: int
        Size of FFT, creates ``n_fft // 2 + 1`` bins. (Default: ``400``)
    win_leght: int
        Window size. (Default: ``n_fft``)
    hop_lenght: int
        Length of hop between STFT windows. (Default: "win_lenght // 2")

    Returns
    -------
    torch.FloatTensor
        Spectrogram in dB of the input waveform
    """
    
    spectrogram = T.Spectrogram(
        n_fft=self.n_fft,
        win_length=self.win_lenght,
        hop_length=self.hop_lenght,
        center=True,
        pad_mode="reflect",
        power=2.0
    )

    return spectrogram(waveform)