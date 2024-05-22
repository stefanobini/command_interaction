import os
import shutil
import random
import matplotlib.pyplot as plt
import math
import librosa
import librosa.display
from typing import List, Tuple
import numpy as np
import itertools

import torch
import torchaudio
import torchaudio.transforms as T
import torchaudio.functional as F

from settings.conf_1 import settings

import colorama
from colorama import Back, Fore
colorama.init(autoreset=True)


class Preprocessing():
    def __init__(self):
        super().__init__()

    
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
        resampler = T.Resample(orig_freq=sample_rate, new_freq=settings.input.sample_rate, dtype=waveform.dtype)
        
        # Lowpass filter width: larger lowpass_filter_width -> more precise filter, but more computationally expensive
        # Rolloff: lower rolloff reduces the amount of aliasing, but it will also reduce some of the higher frequencies
        # Window function
        return resampler(waveform)

    
    def get_melspectrogram(self, waveform:torch.FloatTensor) -> torch.FloatTensor:
        transformation = torchaudio.transforms.MelSpectrogram(sample_rate=settings.input.sample_rate, n_fft=settings.input.spectrogram.n_fft, win_length=settings.input.spectrogram.win_lenght, hop_length=settings.input.spectrogram.hop_lenght, n_mels=settings.input.mel.n_mels)

        return transformation(waveform)
    

    def get_mfcc(self, waveform:torch.FloatTensor) -> torch.FloatTensor:
        mel_arg = {"n_fft":settings.input.spectrogram.n_fft, "win_length":settings.input.spectrogram.win_lenght, "hop_length":settings.input.spectrogram.hop_lenght, "n_mels":settings.input.mel.n_mels}
        transformation = torchaudio.transforms.MFCC(sample_rate=settings.input.sample_rate, n_mfcc=settings.input.mfcc.n_mfcc, dct_type=settings.input.mfcc.dct_type, norm=settings.input.mfcc.norm, log_mels=settings.input.mfcc.log_mels, melkwargs=mel_arg)
        
        return transformation(waveform)


    def power_to_db_spectrogram(self, spectrogram:torch.FloatTensor):
        spectrogram_db = torchaudio.functional.amplitude_to_DB(x=spectrogram, multiplier=10., amin=1e-10, db_multiplier=np.log10(max(spectrogram.max(), 1e-10)).numpy(), top_db=80)
        # spectrogram_db = librosa.power_to_db(spectrogram, ref=np.max)
        return spectrogram_db

    
    def compute_snr(self, epoch:int) -> float:
        # a, b, mu, sigma = 0., 0., 0., 0.
        snr = 0.
        descent_epochs = settings.training.max_epochs*settings.noise.descent_ratio
        if settings.noise.curriculum_learning.distribution == "uniform":
            snr = random.uniform(settings.noise.min_snr, settings.noise.max_snr)
        elif settings.noise.curriculum_learning.distribution == "dynamic_uniform":
            ### CURRICULUM LEARNING ###
            a = epoch * (settings.noise.min_snr - settings.noise.max_snr) / descent_epochs + settings.noise.max_snr                           # modeled as a linear descending function plus a flat phase in the end
            # b = epoch * (settings.noise.min_snr - settings.noise.max_snr) / self.descent_epochs + settings.noise.max_snr + settings.noise.curriculum_learning.uniform.ab_uniform_step    # modeled as a linear descending function plus a flat phase in the end
            b = settings.noise.max_snr
            if b > settings.noise.max_snr:
                b = settings.noise.max_snr
            snr = random.uniform(a=a, b=b)
        elif settings.noise.curriculum_learning.distribution == "dynamic_gaussian":
            ### CURRICULUM LEARNING ###
            # model mu as a combination of descent linear function plus a constant  ->  \_
            mu = epoch * (settings.noise.min_snr - settings.noise.max_snr) / descent_epochs + settings.noise.max_snr     # modeled as a linear descending function plus a flat phase in the end
            # model sigma as a triangular function                                  ->  /\
            s1 = epoch * (-settings.noise.curriculum_learning.gaussian.max_sigma) / settings.training.max_epochs + settings.noise.curriculum_learning.gaussian.max_sigma + settings.noise.curriculum_learning.gaussian.min_sigma      # modeled as a linear descending function
            s2 = epoch * settings.noise.curriculum_learning.gaussian.max_sigma / settings.training.max_epochs + settings.noise.curriculum_learning.gaussian.min_sigma                          # modeled as a linear ascending function
            sigma = min(s1, s2)   
            snr = np.random.normal(loc=mu, scale=sigma)
            if (sigma < settings.noise.curriculum_learning.gaussian.min_sigma or sigma > settings.noise.curriculum_learning.gaussian.max_sigma) and settings.noise.curriculum_learning.gaussian.max_sigma != 0:
                raise Exception("Error on sigma computation: {} [{}, {}]".format(sigma, settings.noise.curriculum_learning.gaussian.min_sigma, settings.noise.curriculum_learning.gaussian.max_sigma))
            
        # pre_snr = snr
        if snr > settings.noise.max_snr:
            snr = settings.noise.max_snr
        elif snr < settings.noise.min_snr:
            snr = settings.noise.min_snr 
        
        # n_sample += 1
        # print("*****\nEpoch: {}/{}\tmu: {}\tsigma: {}\tsnr: {}({})[{}, {}]\n******\n".format(epoch, settings.training.max_epochs, mu, sigma, snr, pre_snr, settings.noise.min_snr, settings.noise.max_snr))
        # print(Back.YELLOW + "*****\nEpoch: {}/{}\ta: {}\tb: {}\tsnr: {}({})[{}, {}]\n******\n".format(epoch, settings.training.max_epochs, a, b, snr, pre_snr, settings.noise.min_snr, settings.noise.max_snr))
        return int(snr)


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
            noise = torch.unsqueeze(noise, dim=0)
        elif noise.size(1) < speech.size(1):
            start = random.randint(a=0, b=speech.size(1)-noise.size(1))
            end = start + noise.size(1)
            t = torch.zeros(size=speech.size())
            t[0, start:end] = noise
            noise = t
        return noise       


    def db_to_float(db, using_amplitude=True):
        """
        Converts the input db to a float, which represents the equivalent
        ratio in power.
        """
        db = float(db)
        if using_amplitude:
            return 10 ** (db / 20)
        else:  # using power
            return 10 ** (db / 10)


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

        speech_rms = speech.norm(p=2)
        noise_rms = noise.norm(p=2)
        
        snr = 10**(snr_db/20)   # before 10
        #'''
        noise_gain = speech_rms / (snr * noise_rms)
        noise_gain = min(noise_gain, settings.input.noise.max_gain)
        noisy_speech = (noise_gain * noise + speech) / 2
        '''
        speech_gain = snr * noise_rms / speech_rms
        noisy_speech = (speech_gain * speech + noise) / 2
        #'''
        # print("speech_power:\t{}, noise_power:\t{}, snr_db:\t{}, snr:\t{}, noise_gain:\t{}".format(speech_power, noise_power, snr_db, snr, noise_gain))
        return noisy_speech


def detect_silence(audio_segment, min_silence_len=1000, silence_thresh=-16, seek_step=1):
    """
    Returns a list of all silent sections [start, end] in milliseconds of audio_segment.
    Inverse of detect_nonsilent()
    audio_segment - the segment to find silence in
    min_silence_len - the minimum length for any silent section
    silence_thresh - the upper bound for how quiet is silent in dFBS
    seek_step - step size for interating over the segment in ms
    """
    seg_len = len(audio_segment)

    # you can't have a silent portion of a sound that is longer than the sound
    if seg_len < min_silence_len:
        return []

    # convert silence threshold to a float value (so we can compare it to rms)
    silence_thresh = db_to_float(silence_thresh) * audio_segment.max_possible_amplitude

    # find silence and add start and end indicies to the to_cut list
    silence_starts = []

    # check successive (1 sec by default) chunk of sound for silence
    # try a chunk at every "seek step" (or every chunk for a seek step == 1)
    last_slice_start = seg_len - min_silence_len
    slice_starts = range(0, last_slice_start + 1, seek_step)

    # guarantee last_slice_start is included in the range
    # to make sure the last portion of the audio is searched
    if last_slice_start % seek_step:
        slice_starts = itertools.chain(slice_starts, [last_slice_start])

    for i in slice_starts:
        audio_slice = audio_segment[i:i + min_silence_len]
        if audio_slice.rms <= silence_thresh:
            silence_starts.append(i)

    # short circuit when there is no silence
    if not silence_starts:
        return []

    # combine the silence we detected into ranges (start ms - end ms)
    silent_ranges = []

    prev_i = silence_starts.pop(0)
    current_range_start = prev_i

    for silence_start_i in silence_starts:
        continuous = (silence_start_i == prev_i + seek_step)

        # sometimes two small blips are enough for one particular slice to be
        # non-silent, despite the silence all running together. Just combine
        # the two overlapping silent ranges.
        silence_has_gap = silence_start_i > (prev_i + min_silence_len)

        if not continuous and silence_has_gap:
            silent_ranges.append([current_range_start,
                                  prev_i + min_silence_len])
            current_range_start = silence_start_i
        prev_i = silence_start_i

    silent_ranges.append([current_range_start,
                          prev_i + min_silence_len])

    return silent_ranges


def ranges2list(ranges):
    indices = list()
    for r in ranges:
        print(r)
        indices.extend(*range(r))
    return indices


def detect_nonsilent(audio_segment, min_silence_len=1000, silence_thresh=-16, seek_step=1):
    """
    Returns a list of all nonsilent sections [start, end] in milliseconds of audio_segment.
    Inverse of detect_silent()
    audio_segment - the segment to find silence in
    min_silence_len - the minimum length for any silent section
    silence_thresh - the upper bound for how quiet is silent in dFBS
    seek_step - step size for interating over the segment in ms
    """
    silent_ranges = detect_silence(audio_segment, min_silence_len, silence_thresh, seek_step)
    len_seg = len(audio_segment)

    # if there is no silence, the whole thing is nonsilent
    if not silent_ranges:
        return [[0, len_seg]]

    # short circuit when the whole audio segment is silent
    if silent_ranges[0][0] == 0 and silent_ranges[0][1] == len_seg:
        return []

    prev_end_i = 0
    nonsilent_ranges = []
    for start_i, end_i in silent_ranges:
        nonsilent_ranges.append([prev_end_i, start_i])
        prev_end_i = end_i

    if end_i != len_seg:
        nonsilent_ranges.append([prev_end_i, len_seg])

    if nonsilent_ranges[0] == [0, 0]:
        nonsilent_ranges.pop(0)

    return nonsilent_ranges


def plot_melspectrogram(path:str, melspectrogram:torch.FloatTensor) -> None:
    fig, ax = plt.subplots()
    #melspectrogram_db = librosa.power_to_db(melspectrogram, ref=np.max)
    melspectrogram_db = melspectrogram[0].numpy()
    img = librosa.display.specshow(melspectrogram_db, y_axis='mel', x_axis='time', ax=ax)
    ax.set(title='Mel spectrogram display')
    fig.colorbar(img, ax=ax, format="%+2.f dB")
    plt.savefig(path)

def plot_mfcc(path:str, mfcc:torch.FloatTensor) -> None:
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
        n_fft=settings.input.spectrogram.n_fft,
        win_length=settings.input.spectrogram.win_lenght,
        hop_length=settings.input.spectrogram.hop_lenght,
        center=True,
        pad_mode="reflect",
        power=2.0
    )

    return spectrogram(waveform)