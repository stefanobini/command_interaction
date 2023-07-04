import os

USER = "450427228"
LANG = "esp"
folder = os.path.join("recordings", USER, LANG)

for file in os.listdir(folder):
    old_file = os.path.join(folder, file)
    new_file = os.path.join(folder, file.replace('.', "_0."))
    os.rename(src=old_file, dst=new_file)