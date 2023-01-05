# Framework for Speech-Command Recognition
In the industrial scenario, the need to control production lines in real-time for production optimization and the customization of the finished product made it necessary to collaborate between humans and robots.
Due to the strong performance and computational constraints required by the environment, collaboration is understood as Command-based Interaction.

In this context, for the verbal communication module, the objective is the definition and implementation of Speech-Command Recognition algorithms that allow a cognitive robot in the industrial field to understand the commands given by a human operator on the production line.

![alt text](https://github.com/stefanobini/command_interaction/training/blob/main/figures/sci_workflow.png)

## Conda environment
Activate conda environment with all the required libraries
```bash
conda activate training
```

## Preliminary operations
Before starting with the training of the system you should create the dataset with command, synthetic and reject samples. Then there are the necessary commands. The source and destination paths need to be changed in the code, along with their parameters.

For command samples you can load them with the right format using the script "create_full_dataset.py":
```bash
python3 create_full_dataset.py
```

For synthetic samples you can copy them from the original dataset:
```bash
cp -r /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/dataset/full_dataset_v1/synthetics /mnt/sdb1/sbini/Speech-Command_Interaction/training/datasets/full_dataset_v1/synthetics
```

For reject samples you can load them with the right format using the scripts "include_common_voice.py" and "include_google_speech.py":
```bash
python3 include_common_voice.py
python3 include_google_speech.py
```

The script "count_commands.py" allow to obtain statistics on the dataset.
```bash
python3 count_commands.py
```

Finally, to split dataset and obtain csv file to train the system use the following comamnd.
```bash
python3 split_dataset.py
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