# Microphone

## Installation

1. Install python3 
2. Install requirements
- Run ``` pip install -r requirements.txt ``` in the working directory
3. Install pyaudio
- If you use windows run: ``` pip install pipwin ``` and then ``` pipwin install pyaudio ```

## Start recording

1. Before starting, specify the following parameters:
- ```-sr SR``` -> sampling rate
-  ```-id_device ID_DEVICE``` -> input device index
2. You can set the following optional parameters:
- ```-recording_time REC_TIME``` -> recording time in seconds, if you desire continuous recording ignore this parameter
- ```-frames_per_buffer FRAMES_PER_BUFFER``` -> number of frames per read, default=1024
- ```-fname FNAME``` -> File name, do not specify the file extension. If this parameter is not specified the software generates the file name automatically

## Examlples
1. Running continuous recording
- ``` python microphone.py -sr 16000 -id_device 1 ``` 
2. Running recording for a given number of seconds
- ``` python microphone.py -sr 16000 -id_device 1 -recording_time 5 ```
3. Setting a file name
- ``` python microphone.py -sr 16000 -id_device 1 -recording_time 5 -fname prova ```

## Notes
1. The software saves the audio data in wav format using a int16 representation
2. When specifying the ```-fname``` parameter, do **not specify** the file extension
3. The json file "subject_info.json" is used to catch the metadata related to the sample
