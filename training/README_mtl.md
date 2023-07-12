# Multi-task Learning between Speech-Command Recognition and Speaker Re-Identification

![alt text](https://github.com/stefanobini/command_interaction/training/blob/main/figures/sci_workflow.png)

## Conda environment
Activate conda environment with all the required libraries
```bash
conda activate TO_BUILD
```

Alternatively you can type the following commands.
```bash
conda create -n <env_name> python=3.9
conda activate <env_name>
python3 -m pip install numpy torch torchvision torchaudio pytorch-lightning pandas colorama tqdm librosa matplotlib dotmap torchmetrics torchsummary tensorboard python-telegram-bot==13.7
```

## Libraries and frameworks
- cuda == 11.7
- python == 3.9.16
- numpy == 1.23.5
- torch == 2.0.0+cu117
- torchvision == 0.15.1+cu117
- torchaudio == 2.0.1+cu117
- pytorch-lightning == 2.0.0
- tensorboard == 
- librosa == 0.10.0.post2

## Preliminary operations
Before starting with the training of the system you should create the dataset with command, synthetic and reject samples. Then there are the necessary commands. The source and destination paths need to be changed in the code, along with their parameters.

### Dataset and annotation file for SCR and SID
To create annotation files:
```bash
python3 datasets/scripts/remove_underrepresented_speakers.py
```

After moving the previous annotation file in the new experiemntation folder and renamed it in "dataset_no_speaker_id.csv", use the following command to convert the name of the speaker in an ID:
```bash
python3 datasets/scripts/fix_speaker_id.py
```

To split dataset in training and validation set for training SCR-SID multitask network:
```bash
python3 datasets/scripts/split_annotations_MTL_train.py
```

To check if training and validation contain the same speakers:
```bash
python3 datasets/scripts/check_speakers.py
```

To build the dataset (audio file):
```bash
python3 datasets/scripts/build_dataset.py
```

### Dataset and annotation file for SCR and SRID
After moving the previous annotation file in the new experiemntation folder and renamed it in "dataset_no_speaker_id.csv", use the following command to convert the name of the speaker in an ID:
```bash
python3 datasets/scripts/fix_speaker_id.py
```

To split dataset in training and validation set for training SCR-SID multitask network:
```bash
python3 datasets/scripts/split_annotations_MTL_knn.py
```

## Dataset and annotation files for SCR and SR

To remove under-represented speakers from the annotation files:
```bash
python3 datasets/scripts/SCR_SR_remove_underrepresented_speakers.py
```

To convert the name of the speaker in an ID:
```bash
python3 datasets/scripts/fix_speaker_id.py
```

To create annotation files for training, validation, and testing phase:
```bash
python3 datasets/scripts/SCR_SR_split_annotations.py
```


# Training phase
All the information for training phase are included in a configuration file placed in the "./settings/" folder. You can create more configuration files and select the one to use passing it as parameter in the following command (only name, without ".py").
```bash
python3 train.py --configuration MTL_conf
```
