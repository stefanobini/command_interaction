from pathlib import Path
import torch
import pytorch_lightning as pl
from omegaconf import OmegaConf
from utils import Logger
from global_utils import get_curr_dir
from commands import command_eng, command_ita
from model import Augmentation, ValidationCallback, Model
import subprocess
import platform
import os
import argparse

try:
    from google.colab import drive
    COLAB = True
except Exception:
    COLAB = False


'''
To train change the dataset path here and in <model.py>, then change commands in <commands.py>
Parsing input argument
python3 matchboxnet.py --dataset_path dataset/FELICE_demo3 --config ./matchboxnet_3x2x64_FELICE.yaml --lang ita --id 0 --log 0 --synth 1 --pre_train ./pretrain_models
python3 matchboxnet.py --dataset_path dataset/FELICE_demo7_extended --config ./matchboxnet_3x2x64_FELICE.yaml --lang ita --id 0 --log 0 --synth 1 --pre_train ./pretrain_models
python3 matchboxnet.py --dataset_path dataset/full_dataset_v1 --config ./matchboxnet_3x2x64_no_noise.yaml --lang ita --id 0 --log 0 --synth 1
python3 matchboxnet.py --dataset_path dataset/full_dataset_v1 --config ./matchboxnet_3x2x64_full.yaml --lang ita --id 0 --log 0 --synth 1

python3 matchboxnet.py --dataset_path dataset/test_dataset --config ./matchboxnet_3x2x64_full.yaml --lang ita --id 0 --log 0 --synth 0
'''


if __name__ == "__main__":
<<<<<<< HEAD
    '''
    Parsing input argument
    python3 matchboxnet.py --dataset_path dataset/FELICE_demo3 --config ./matchboxnet_3x2x64_FELICE.yaml --lang ita --id 0 --log 0 --synth 1 --pre_train ./pretrain_models
    python3 matchboxnet.py --dataset_path dataset/FELICE_demo7_phase_I --config ./matchboxnet_3x2x64_FELICE.yaml --lang eng --id 0 --log 0 --synth 1 --pre_train ./pretrain_models
    '''
=======
    
>>>>>>> prova
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, dest="config", required=True, help="Yaml file containing the configuration")
    parser.add_argument("--lang", type=str, dest="lang", required=False, default="eng", help='Desired lang')
    parser.add_argument("--id", type=str, required=True, dest="train_id", help='training id')
    parser.add_argument("--log", type=int, required=False, dest="logfile", default=1, help='0 to disable log file, 1 to activate log file, default 1')
    parser.add_argument("--synth", type=int, required=False, dest="synth", default=1, help='0 to exculde synthetic samples, 1 to include it, default 1')
    parser.add_argument("--pre_train", type=str, default=None, dest="pre_train", help='Path of the pre-training model from which to start')
    parser.add_argument("--dataset_path", type=str, default=None, dest="dataset_path", help='Path of the dataset folder')
    # parser.add_argument("--pre_train", type=int, required=False, dest="pre_train", default=0, help='1 if the user is loading a model, 0 (default) if he is creating the model')
    args = parser.parse_args()

    if args.logfile:
        logger = Logger()
    
    from nemo.collections.asr.parts.preprocessing.perturb import register_perturbation
    from nemo.utils.exp_manager import exp_manager

    print("*"*30)
    print("Training id", args.train_id)
    print("*" * 30)

    # Select the computing device
    if not COLAB and platform.system().lower() != "windows":
        os.environ["CUDA_VISIBLE_DEVICES"] = str(0)     # <------ GPU device
    
    register_perturbation("mynoise", Augmentation)
    path_base = Path(get_curr_dir(__file__)).joinpath("manifests")

    ##Variables
    LANG = args.lang.lower()
    assert LANG == "eng" or LANG == "ita"
    TRAIN_MANIFEST = str(path_base.joinpath(f"train_{LANG}_manifest.json"))

    ##Creating config file
    config_path = Path(get_curr_dir(__file__)).joinpath("manifests", args.config)
    config = OmegaConf.load(config_path)
    config = OmegaConf.to_container(config, resolve=False)
    config = OmegaConf.create(config)
    LABELS = list(map(lambda x: int(x), command_eng.keys()))
    LABELS.append(int(len(command_eng)))  # if commented it does not consider reject option
    batch_size = config.model.train_ds.batch_size
    config.model.labels = LABELS
    config.model.lang = LANG

    config.model.train_ds.augmentor.mynoise.dataset_path_real = str(Path(get_curr_dir(__file__)).joinpath("{}/commands".format(args.dataset_path)))
    if args.synth:
        config.model.train_ds.augmentor.mynoise.dataset_path_synth = str(Path(get_curr_dir(__file__)).joinpath("{}/synthetics".format(args.dataset_path)))
    else:
        config.model.train_ds.augmentor.mynoise.dataset_path_synth = None
    config.model.train_ds.augmentor.mynoise.dataset_path_reject = str(Path(get_curr_dir(__file__)).joinpath("{}/rejects".format(args.dataset_path)))

    config.model.train_ds.manifest_filepath = TRAIN_MANIFEST

    OmegaConf.resolve(config)
    sample_rate = config.model.sample_rate
    print("##### Trainer config #####")
    print(OmegaConf.to_yaml(config.trainer))
    print("##########################\n")

    cuda = 1 if torch.cuda.is_available() else 0
    config.trainer.gpus = cuda
    config.trainer.accelerator = None

    validation_callback = ValidationCallback(config.model.train_ds.augmentor.mynoise, config.model.labels, batch_size, args.train_id)
    CALLBACKS = [validation_callback]
    # CALLBACKS = []
    trainer = pl.Trainer(**config.trainer, callbacks=CALLBACKS)

    asr_model = None
    if args.pre_train:
        if 'nemo_model' in args.pre_train:
            asr_model = Model.load_backup(trainer=trainer, ckpt_name='commandrecognition_en_matchboxnet3x2x64_v2.nemo', exp_dir=args.pre_train, new_cfg=config.model)
            asr_model.change_labels(config.model.labels)
            
            asr_model.cuda()
            
            '''
            asr_model = nemo_asr.models.EncDecClassificationModel.from_pretrained(model_name="commandrecognition_en_matchboxnet3x2x64_v2")

            asr_model.change_labels(config.model.labels)
            asr_model.setup_training_data(config.model.train_ds)
            asr_model._update_decoder_config(labels=config.model.labels, cfg=config.model.decoder)
            asr_model.setup_optimization(optim_config=config.model.optim)
            asr_model.set_trainer(trainer=trainer)
            '''
        else:
            # pretrain_path = os.path.join(args.pre_train, args.lang+'.model')
            # pretrain_config = copy.deepcopy(config)
            # pretrain_config.model.labels = [i for i in range(0, 29)]
            # asr_model = Model(cfg=config.model, trainer=trainer, class_weight=None) #Modify here to set class weight
            # asr_model = asr_model.load_from_checkpoint(checkpoint_path=pretrain_path)
            asr_model = Model.load_backup(trainer=trainer, ckpt_name=args.lang+'.model', exp_dir=args.pre_train, new_cfg=config.model)  # source code of the model here: <https://docs.nvidia.com/deeplearning/nemo/user-guide/docs/en/stable/_modules/nemo/collections/asr/models/classification_models.html#EncDecClassificationModel>
            asr_model.change_labels(config.model.labels)
            
            asr_model.cuda()
            # asr_model.device('cuda:{}'.format(os.environ["CUDA_VISIBLE_DEVICES"]))
    else:
        asr_model = Model(cfg=config.model, trainer=trainer, class_weight=None) #Modify here to set class weight
    
    with open('info_model.txt', 'w') as f:
        f.write(str(asr_model))
        f.write(str(asr_model.train_dataloader))
        f.write(str(asr_model.cfg))
    
    asr_model._setup_dataloader_from_config(asr_model.cfg.train_ds)

    exp_dir = exp_manager(trainer, config.get("exp_manager", None))
    exp_dir = str(exp_dir)
    if args.logfile:
        logger.exp_dir = exp_dir
    print(exp_dir)
    validation_callback.exp_dir = exp_dir

    if platform.system().lower() == "windows":
        logdir = Path(exp_dir).relative_to(get_curr_dir(__file__))
        cmd = ["tensorboard", "--logdir", logdir]
        p = subprocess.Popen(cmd)

    ##Training
    print(f"Starting training for {LANG}\n")
    trainer.fit(asr_model)

    if args.logfile:
        logger.close()