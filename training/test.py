import torchaudio
import torchaudio.transforms as T
import torch
import matplotlib.pyplot as plt
import librosa
import librosa.display
import numpy as np
import math
import random
from colorama import Back


def get_melspectrogram(waveform:torch.FloatTensor) -> torch.FloatTensor:
        transformation = torchaudio.transforms.MelSpectrogram(sample_rate=16000, n_fft=512, win_length=400, hop_length=200, n_mels=64)

        return transformation(waveform)

def plot_melspectrogram(path:str, melspectrogram:torch.FloatTensor) -> None:
    fig, ax = plt.subplots()
    melspectrogram_db = librosa.power_to_db(melspectrogram, ref=np.max)
    img = librosa.display.specshow(melspectrogram_db, y_axis='mel', x_axis='time', ax=ax)
    ax.set(title='Mel spectrogram display')
    fig.colorbar(img, ax=ax, format="%+2.f dB")
    plt.savefig(path)

def fix_noise_duration(speech:torch.FloatTensor, noise:torch.FloatTensor) -> torch.FloatTensor:
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

def get_noisy_speech(speech:torch.FloatTensor, noise:torch.FloatTensor, snr_db:float) -> torch.FloatTensor:
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
        
        noise = fix_noise_duration(speech=speech, noise=noise)

        speech_power = speech.norm(p=2)
        noise_power = noise.norm(p=2)
        
        snr = math.exp(snr_db / 10)
        scale = speech_power / (snr * noise_power)
        return (scale * noise + speech) / 2

def resample_audio(waveform:torch.FloatTensor, sample_rate:int) -> torch.FloatTensor:
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
    resampler = T.Resample(orig_freq=sample_rate, new_freq=16000, dtype=waveform.dtype)
    
    # Lowpass filter width: larger lowpass_filter_width -> more precise filter, but more computationally expensive
    # Rolloff: lower rolloff reduces the amount of aliasing, but it will also reduce some of the higher frequencies
    # Window function
    return resampler(waveform)


full_path = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/final_dataset/training/commands/1202998179/ita/ita_21.wav"
noise_path = "/mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1/noises/drill/89099.wav"
out_path = "test.png"
speech, speech_sample_rate = torchaudio.load(full_path)
speech = resample_audio(waveform=speech, sample_rate=speech_sample_rate)  # uniform sample rate
speech = torch.mean(input=speech, dim=0, keepdim=True)  # reduce to one channel

noise, noise_sample_rate = torchaudio.load(filepath=noise_path)
noise = resample_audio(waveform=noise, sample_rate=noise_sample_rate)     # uniform sample rate
noise = torch.mean(input=noise, dim=0, keepdim=True)    # reduce to one channel

waveform = get_noisy_speech(speech=speech, noise=noise, snr_db=40.)

#print(speech.size(), noise.size(), torch.max(waveform), torch.min(waveform), torch.max(speech), torch.min(speech), torch.max(noise), torch.min(noise))
vet = noise[:, 170469:184599]
print(torch.min(vet), torch.max(vet))

if True in torch.isnan(waveform):
    print(Back.RED + "{} with size {} has {} 'nan' value".format(full_path, waveform.size(), torch.count_nonzero(torch.isnan(waveform))))
    print(torch.max(waveform), torch.min(waveform), torch.max(speech), torch.min(speech), torch.max(noise), torch.min(noise))
if True in torch.isinf(waveform):
    print(Back.RED + "{} has {} 'inf' value".format(full_path, waveform.size(), torch.count_nonzero(torch.isinf(waveform))))

melspectrogram = get_melspectrogram(waveform=waveform)
plot_melspectrogram(path=out_path, melspectrogram=melspectrogram[0])