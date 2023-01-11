import os
import shutil
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


def plot_melspectrogram(melspectrogram:torch.FloatTensor) -> None:
    fig, ax = plt.subplots()
    melspectrogram_db = librosa.power_to_db(melspectrogram, ref=np.max)
    img = librosa.display.specshow(melspectrogram_db, y_axis='mel', x_axis='time', ax=ax)
    ax.set(title='Mel spectrogram display')
    fig.colorbar(img, ax=ax, format="%+2.f dB")
    path = os.path.join("figures", "melspectrogram_example.png")
    plt.savefig(path)


def resample_audio(waveform:torch.FloatTensor, sample_rate:int, resample_rate:int, dtype=torch.FloatTensor) -> torch.FloatTensor:
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

    T.Resample(sample_rate, resample_rate, dtype)
    
    # Lowpass filter width: larger lowpass_filter_width -> more precise filter, but more computationally expensive
    # Rolloff: lower rolloff reduces the amount of aliasing, but it will also reduce some of the higher frequencies
    # Window function
    return F.resample(waveform=waveform, orig_freq=sample_rate, new_freq=resample_rate, lowpass_filter_width=6, rolloff=0.99, resampling_method="sinc_interpolation")

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

## Adding background noise
def get_noisy_speech(speech:torch.FloatTensor, noise:torch.FloatTensor, speech_sample_rate:int=16000, noise_sample_rate:int=16000, snr_db:int=100) -> torch.FloatTensor:
    """Apply a noise with specific SNR to a speech waveform.

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
    SNRs: int
        SNRs (in dB) to apply on the speech waveform

    Returns
    -------
    torch.FloatTensor
        Noisy speeches.
    """
    
    noise = resample_audio(waveform=noise, sample_rate=noise_sample_rate, resample_rate=speech_sample_rate)

    speech_power = speech.norm(p=2)
    noise_power = noise.norm(p=2)

    snr = math.exp(snr_db / 10)
    scale = snr * noise_power / speech_power
    return (scale * speech + noise) / 2

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

# FEATURE EXTRACTIONS
## Spectrogram: To get the frequency make-up of an audio signal as it varies with time, you can use Spectrogram.
def get_spectrogram(waveform:torch.FloatTensor, n_fft:int=400, win_lenght:int=400, hop_lenght:int=200) -> torch.FloatTensor:
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
        n_fft=n_fft,
        win_length=win_lenght,
        hop_length=hop_lenght,
        center=True,
        pad_mode="reflect",
        power=2.0
    )

    return spectrogram(waveform)