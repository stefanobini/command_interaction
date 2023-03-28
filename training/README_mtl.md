# Multi-task Learning between Speech-Command Recognition and Speaker Re-Identification

![alt text](https://github.com/stefanobini/command_interaction/training/blob/main/figures/sci_workflow.png)

## Conda environment
Activate conda environment with all the required libraries
```bash
conda activate training
```

## Libraries and frameworks
- cuda == 10.2
- numpy == 1.23.4
- torch == 1.13.1
- torchaudio == 0.13.1
- pandas == 1.5.2
- matplotlib == 3.6.2
- pillow == 9.3.0
- librosa == 0.9.2
- scikit-learn == 1.1.3
- numba == 0.56.4
- requests == 2.28.1
- pysoundfile == 0.11.0
- scipy == 1.9.3

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