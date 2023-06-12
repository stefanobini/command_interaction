import os


FOLDER = "audio_samples/esp"
for file in os.listdir(FOLDER):
    if ".opus" in file:
        name = os.path.join(FOLDER, file.replace(".opus", ""))
        cmd = "ffmpeg -i {}.opus -vn {}.wav".format(name, name)
        os.system(command=cmd)
        cmd = "rm {}.opus".format(name)
        os.system(command=cmd)