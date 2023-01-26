"""
ITA - Synthetics - [-10; 40] - Classic
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/2022-12-02_00-56-53 --ckpt matchcboxnet--val_loss=4.5978-epoch=97.model

ITA - Synthetics - [-10; 40] - PEM
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/2022-12-01_21-41-16 --ckpt matchcboxnet--val_loss=3.113-epoch=86.model

ITA - Synthetics - [-10; 40] - Uniform CL-PEM v1
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/2022-12-01_10-15-22 --ckpt matchcboxnet--val_loss=3.7649-epoch=99.model

ITA - Synthetics - [-10; 40] - Uniform CL-PEM v2
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/2022-12-01_17-51-49 --ckpt matchcboxnet--val_loss=3.8024-epoch=97.model

ITA - Synthetics - [-10; 40] - Gaussian CL-PEM v1
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/2022-12-01_21-35-26 --ckpt matchcboxnet--val_loss=3.269-epoch=97.model

ITA - Synthetics - [-10; 40] - Gaussian CL-PEM v2
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/2022-12-05_12-54-59 --ckpt matchcboxnet--val_loss=3.589-epoch=97.model
"""

"""
ENG - Synthetics - [-10; 40] - Gaussian CL-PEM v1
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/2022-12-06_14-05-40 --ckpt matchcboxnet--val_loss=3.5196-epoch=99.model
"""

################ FIX THE BATCH BALANCING ################
"""
ITA - Synthetics - [-10; 40] - Classic
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/ --ckpt .model

ITA - Synthetics - [-10; 40] - PEM
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/ --ckpt .model

ITA - Synthetics - [-10; 40] - Uniform CL-PEM v1
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/ --ckpt .model

ITA - Synthetics - [-10; 40] - Uniform CL-PEM v2
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/ --ckpt .model

ITA - Synthetics - [-10; 40] - Gaussian CL-PEM v1
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/2022-12-12_15-30-44 --ckpt matchcboxnet--val_loss=0.9453-epoch=74.model

ITA - Synthetics - [-10; 40] - Gaussian CL-PEM v2
python3 matchboxnet_test.py --exp_dir /mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/2022-12-13_12-08-57 --ckpt matchcboxnet--val_loss=0.9415-epoch=85.model
"""


import os
import shutil

import pandas as pd
from pathlib import Path
import torch
from model import Model, ValidationCallback, Augmentation
import pytorch_lightning as pl
from global_utils import get_curr_dir
from omegaconf import OmegaConf
import omegaconf
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
import matplotlib.pyplot as plt
import numpy as np
from sklearn.metrics import accuracy_score, balanced_accuracy_score, classification_report
import commands
from plot_tf_event import PlotManager
import pickle
from utils import is_windows
# from pytimedinput import timedInput
import argparse
from nemo.collections.asr.parts.preprocessing.perturb import register_perturbation


THRESH = 0.90


def save_predicts(predict_labels: dict, true_labels: dict):
    data = (predict_labels, true_labels)
    with open(str(predict_file_path), "wb") as fil:
        pickle.dump(data, fil)

def load_predict() -> np.ndarray:
    with open(str(predict_file_path), "rb") as fil:
        return pickle.load(fil)

def reject_acc(y_true, y_pred):
    assert len(commands.command_eng) == len(commands.command_ita)
    y_true = np.where(y_true == len(commands.command_eng), 0, 1)
    y_pred = np.where(y_pred == len(commands.command_eng), 0, 1)
    return accuracy_score(y_true, y_pred)

def predict_sigmoid(logits, limit=0.0):
    sigmoid = torch.nn.Sigmoid()
    predict_labels = []
    for logits_row in logits:
        out_list = []
        for logit in logits_row:
            out = sigmoid(logit).item()
            out_list.append(out)
        out_list = np.array(out_list)
        mask: np.ndarray = out_list >= limit
        if not mask.any():
            predict_labels.append(len(commands.command_eng))
        else:
            predict_labels.append(np.argmax(out_list))
    return np.array(predict_labels)

def report(true_labels, predict_labels, snr, seed):
    report = classification_report(true_labels, predict_labels, labels=labels)
    path = Path(exp_dir).joinpath(f"res/report_{snr}dB.txt")
    fil = open(str(path), "a")
    print(f"\t{ckpt_name}\n\tsnr: {snr} - seed: {seed}\n{report}", file=fil)
    fil.close()

def remove_rejection(preds, targs):
    new_preds, new_targs = list(), list()
    for idx in range(len(preds)):
        if preds[idx] != len(commands.command_eng) and targs[idx] != len(commands.command_eng):
            new_preds.append(preds[idx])
            new_targs.append(targs[idx])
        '''
        elif targs[idx] != len(commands.command_eng):
            new_preds.append(targs[idx]-1 if targs[idx]!=0 else targs[idx]+1)
            new_targs.append(targs[idx])
            count_rjt_fp += 1
        elif preds[idx] != len(commands.command_eng):
            new_preds.append(preds[idx])
            new_targs.append(preds[idx]-1 if preds[idx]!=0 else preds[idx]+1)
            count_rjt_fn += 1
        else:
            count_rjt_tp += 1
        '''
    new_preds = np.array(new_preds)
    new_targs = np.array(new_targs)
    return new_preds, new_targs


def metrics(true_labels, predict_labels, seed, snr):
    # predict_labels, true_labels = remove_rejection(preds=predict_labels, targs=true_labels)
    # predict_labels = apply_thresh(predict_labels)
    acc = accuracy_score(y_true=true_labels, y_pred=predict_labels)
    rej_acc = reject_acc(true_labels, predict_labels)
    bal_acc = balanced_accuracy_score(y_true=true_labels, y_pred=predict_labels)
    stat_data = {"seed": int(seed), "snr": int(snr), "accuracy": acc, "bal_acc": bal_acc, "rejcet_acc": rej_acc}
    return stat_data

def is_test():
    if predict_file_path.exists():
        # userText, timedOut = timedInput("Do you want re-test? [y/n]:")
        res = input("Do you want re-test? [y/n]:")
        if res.lower() == "y":
            pass
        else:
            return False
    res_path = Path(exp_dir).joinpath("res")
    if res_path.exists():
        try:
            shutil.rmtree(res_path)
        except OSError:
            pass
    res_path.mkdir(parents=True, exist_ok=True)
    return True


def apply_thresh(preds):
    preds_lab_max = np.argmax(preds, axis=1)
    final_preds = preds_lab_max
    preds_val_max = np.max(preds, axis=1)

    for i in range(len(preds_val_max)):
        if preds_val_max[i] < THRESH:
            final_preds[i] = 31
    return final_preds


if __name__ == "__main__":
    
    PLOT = True     # If "False" we can test only snr for wich the system was trained due to Tensorflow plot
    SNR_MIN = -10    # -10
    SNR_MAX = 40    # 20
    SNR_STEP = 5
    NUM_CPU = 6
    seed_list  = range(0, 1, 1)
    snr_range = range(SNR_MIN, SNR_MAX+SNR_STEP, SNR_STEP)

    working_dir = Path(get_curr_dir(__file__)).joinpath(".working_dir/test")
    stats = pd.DataFrame(columns=["seed", "snr", "accuracy", "bal_acc", "rejcet_acc"])
    
    if not is_windows():
        os.environ["CUDA_VISIBLE_DEVICES"] = str(0)
        print("CUDA computation is available!")
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--exp_dir", type=str, required=False, default=None, dest="exp_dir")
    parser.add_argument("--ckpt", type=str, required=False, default=None, dest="ckpt")
    args = parser.parse_args()

    if args.exp_dir is None:
        exp_dir = r"C:\MIE CARTELLE\PROGRAMMAZIONE\GITHUB\tesi_magistrale\nemo_experiments\MatchboxNet-3x2x64\2022-01-19_19-42-14"
    else:
        exp_dir = args.exp_dir
    if not is_windows():
        exp_dir = Path(exp_dir).as_posix()
    if args.ckpt is None:
        ckpt_name = r"matchcboxnet--val_loss=0.492-epoch=145.model"
    else:
        ckpt_name  = args.ckpt
    ckpt_name = str(Path(ckpt_name).name)
    cfg_path = Path(exp_dir).joinpath("hparams.yaml")
    print("*"*50)
    print("exp_dir:", exp_dir)
    print("ckpt_name:", ckpt_name)
    print("*"*50)

    config = OmegaConf.load(cfg_path)
    config = OmegaConf.create(config)
    lang = config.lang
    test_manifest_path = Path(exp_dir).joinpath(rf"db/test_{lang}_manifest.csv")
    noise_test_path = Path(exp_dir).joinpath(rf"db/noise_test_{lang}_manifest.csv")
    # ckpt = Path(exp_dir).joinpath("checkpoints", ckpt_name)
    predict_file_path = Path(exp_dir).joinpath("res/predict.data")

    try:
        SR = config.sample_rate
    except omegaconf.errors.ConfigAttributeError:
        config = config.cfg
        SR = config.sample_rate

    register_perturbation("mynoise", Augmentation)

    # asr_model: Model = Model.load_from_checkpoint(str(ckpt), last_layer="softmax")
    # asr_model = Model.load_backup(ckpt_name, exp_dir)
    batch_size = config["train_ds"]["batch_size"]
    sample_rate = config["sample_rate"]
    labels = config["labels"]
    max_gain_db = config.train_ds.augmentor.mynoise.max_gain_db

    test = is_test()
    if PLOT:
        plot_manager = PlotManager(exp_dir, SNR_MIN, SNR_MAX, SNR_STEP)
        plot_manager.plot_all()
    for i, seed in enumerate(seed_list):
        #'''
        validation_callback = ValidationCallback(labels, batch_size, snr_range,
                                                 sample_rate, lang, seed, max_gain_db,
                                                 noise_test_path, test_manifest_path, test=True)
        #'''
        # validation_callback = ValidationCallback(config.model.train_ds.augmentor.mynoise, config.model.labels, batch_size, args.train_id)

        # CALLBACKS = [validation_callback]
        # trainer = pl.Trainer(**config.trainer, callbacks=CALLBACKS)
        asr_model = Model.load_backup(trainer=None, ckpt_name=ckpt_name, exp_dir=exp_dir, new_cfg=config)

        dataloaders = validation_callback.dataloaders
        predict_db, true_labels_db, pred_no_argmax_db = dict(), dict(), dict()
        for snr, dataloader in dataloaders.items():
            if test:
                try:
                    _, true_labels, predict_labels, pred_no_argmax, logits = validation_callback.validate(asr_model, dataloader, device="gpu")
                except RuntimeError:
                    _, true_labels, predict_labels, pred_no_argmax, logits = validation_callback.validate(asr_model, dataloader, device="cpu")
                predict_labels = apply_thresh(pred_no_argmax)
                predict_db[snr] = predict_labels
                true_labels_db[snr] = true_labels
                pred_no_argmax_db[snr] = pred_no_argmax
                save_predicts(predict_db, true_labels_db)
            else:
                predict_db, true_labels_db = load_predict()
                with open("test_data_distribution.json", "a") as f:
                    for snr, sample in true_labels_db.items():
                        f.write("***SNR: {}***\n".format(snr))
                        for elem in sample:
                            f.write("{}, ".format(elem))
                        f.write("\n\n")
                predict_labels, true_labels = predict_db[snr], true_labels_db[snr]
            stat_data = metrics(true_labels, predict_labels, seed=seed, snr=snr)
            print(stat_data)
            report(true_labels, predict_labels, snr, seed)
            if i == 0:
                matrix = confusion_matrix(true_labels, y_pred=predict_labels, labels=labels)
                disp = ConfusionMatrixDisplay(confusion_matrix=matrix, display_labels=labels)
                disp.plot()
                plt.title(f"snr: {snr} - seed: {seed}")
                if PLOT:
                    plot_manager.save_confusion_matrix(plt.gcf(), snr)
            stats = stats.append(stat_data, ignore_index=True)
        stats.sort_values(by=["snr", "seed"], inplace=True)
        print(stats)
    print(stats)
    out = Path(get_curr_dir(__file__)).joinpath(f"{exp_dir}/res")
    out.mkdir(exist_ok=True)
    out = str(out.joinpath("stats.csv"))
    print("Stats saved to", out)
    stats.to_csv(out, index=False)