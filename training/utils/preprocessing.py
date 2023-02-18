import os
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
    """Class that preprocesses audio samples before sending them to the network.
    
    Methods
    -------
    resample_audio(self, waveform:torch.Tensor, sample_rate:int) -> torch.Tensor
        Resample an audio waverform given as input (waveform). The target sample rate has to be defined in configuration file, while the original one is passed as parameter (sample_rate)
    get_melspectrogram(self, waveform:torch.Tensor) -> torch.Tensor
        Compute Mel-Spectrogram of a waveform given as input (waveform). All the parameters (sample_rate, n_fft, win_length, hop_length, n_mels) are taken from the configuration file.
    get_mfcc(self, waveform:torch.Tensor) -> torch.Tensor
        Compute MFCC of a waveform given as input (waveform). All the parameters (sample_rate, n_fft, win_length, hop_length, n_mels, n_mfcc, dct_type, norm, log_mels) are taken from the configuration file.
    power_to_db_spectrogram(self, spectrogram:torch.Tensor) -> torch.Tensor
        Convert power spectrogram in dB spectrogram.
    approximate_snr(self, snr:float, multiple:int) -> int
        Approximate SNR to have a pool of values grouped in multiples of "multiple" to allow for performance evaluation
    """
    def __init__(self):
        super().__init__()

    
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
        resampler = T.Resample(orig_freq=sample_rate, new_freq=settings.input.sample_rate, dtype=waveform.dtype)
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
        transformation = torchaudio.transforms.MelSpectrogram(sample_rate=settings.input.sample_rate, n_fft=settings.input.spectrogram.n_fft, win_length=settings.input.spectrogram.win_length, hop_length=settings.input.spectrogram.hop_length, n_mels=settings.input.mel.n_mels)
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
        mel_arg = {"n_fft":settings.input.spectrogram.n_fft, "win_length":settings.input.spectrogram.win_length, "hop_length":settings.input.spectrogram.hop_length, "n_mels":settings.input.mel.n_mels}
        transformation = torchaudio.transforms.MFCC(sample_rate=settings.input.sample_rate, n_mfcc=settings.input.mfcc.n_mfcc, dct_type=settings.input.mfcc.dct_type, norm=settings.input.mfcc.norm, log_mels=settings.input.mfcc.log_mels, melkwargs=mel_arg)
        return transformation(waveform)


    def power_to_db_spectrogram(self, spectrogram:torch.Tensor) -> torch.Tensor:
        """Convert power spectrogram in dB spectrogram.
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
        spectrogram_db = torchaudio.functional.amplitude_to_DB(x=spectrogram, multiplier=10., amin=1e-10, db_multiplier=np.log10(max(spectrogram.max(), 1e-10)).numpy(), top_db=80)
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
        """Calcola SNR da applicare allo specifico campione audio data l'epoca.
        Il metodo usa l'epoca per applicare la strategia di Curriculum Learning (CL) desiderata tra: uniforme (senza CL), gaussiana con varianza fissata e media decrescente, gaussiana con varianza variabile e media decrescente
        """
        # a, b, mu, sigma = 0., 0., 0., 0.
        snr = 0.
        descent_epochs = settings.training.max_epochs*settings.noise.descent_ratio
        if settings.noise.curriculum_learning.distribution == "PEM":
            snr = random.uniform(settings.noise.min_snr, settings.noise.max_snr)
        # TO DO
        elif settings.noise.curriculum_learning.distribution == "UniCL_PEM_v1":
            ### CURRICULUM LEARNING ###
            a = epoch * (settings.noise.min_snr - settings.noise.max_snr) / descent_epochs + settings.noise.max_snr                           # modeled as a linear descending function plus a flat phase in the end
            # b = epoch * (settings.noise.min_snr - settings.noise.max_snr) / self.descent_epochs + settings.noise.max_snr + settings.noise.curriculum_learning.uniform.ab_uniform_step    # modeled as a linear descending function plus a flat phase in the end
            b = settings.noise.max_snr
            if b > settings.noise.max_snr:
                b = settings.noise.max_snr
            snr = random.uniform(a=a, b=b)
        # TO DO
        elif settings.noise.curriculum_learning.distribution == "UniCL_PEM_v1":
            pass
        # TO DO
        elif settings.noise.curriculum_learning.distribution == "GaussCL_PEM_v2":
            mu = epoch * (settings.noise.min_snr - settings.noise.max_snr) / descent_epochs + settings.noise.max_snr     # modeled as a linear descending function plus a flat phase in the end
            sigma = None
            pass
        elif settings.noise.curriculum_learning.distribution == "GaussCL_PEM_v1":
            ### CURRICULUM LEARNING ###
            # model mu as a combination of descent linear function plus a constant  ->  \_
            mu = epoch * (settings.noise.min_snr - settings.noise.max_snr) / descent_epochs + settings.noise.max_snr     # modeled as a linear descending function plus a flat phase in the end
            # model sigma as a triangular function                                  ->  /\
            #s1 = epoch * (-settings.noise.curriculum_learning.gaussian.max_sigma) / settings.training.max_epochs + settings.noise.curriculum_learning.gaussian.max_sigma + settings.noise.curriculum_learning.gaussian.min_sigma      # modeled as a linear descending function
            #s2 = epoch * settings.noise.curriculum_learning.gaussian.max_sigma / settings.training.max_epochs + settings.noise.curriculum_learning.gaussian.min_sigma                          # modeled as a linear ascending function
            # model sigma as a triangular function                                  ->  \/
            s1 = epoch * (-settings.noise.curriculum_learning.gaussian.max_sigma) / settings.training.max_epochs      # modeled as a linear ascending function
            s2 = epoch * settings.noise.curriculum_learning.gaussian.max_sigma / settings.training.max_epochs + settings.noise.curriculum_learning.gaussian.max_sigma                          # modeled as a linear descending function
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


    def get_noisy_speech(self, speech:torch.Tensor, noise:torch.Tensor, snr_db:float) -> torch.Tensor:
        """Apply a noise with specific SNR to a speech waveform.

        Parameters
        ----------
        speech: torch.Tensor
            Speech waveform
        noise: torch.Tensor
            Noise waveform
        snr: float
            SNR in dB applied to the speech sample

        Returns
        -------
        torch.Tensor
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


def plot_melspectrogram(path:str, melspectrogram:torch.Tensor) -> None:
    fig, ax = plt.subplots()
    #melspectrogram_db = librosa.power_to_db(melspectrogram, ref=np.max)
    melspectrogram_db = melspectrogram[0].numpy()
    img = librosa.display.specshow(melspectrogram_db, y_axis='mel', x_axis='time', ax=ax)
    ax.set(title='Mel spectrogram display')
    fig.colorbar(img, ax=ax, format="%+2.f dB")
    plt.savefig(path)

def plot_mfcc(path:str, mfcc:torch.Tensor) -> None:
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