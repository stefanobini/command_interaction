import tensorflow as tf
import numpy
import os
from tqdm import tqdm


FOLDER = "/mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/2022-12-12_15-30-44"
CLASS_FILE = "class_distributions.txt"
CMDS_FILE = "commands_distributions.txt"
RJT_FILE = "reject_distributions.txt"

class_file_path = os.path.join(FOLDER, CLASS_FILE)
cmds_file_path = os.path.join(FOLDER, CMDS_FILE)
rjt_file_path = os.path.join(FOLDER, RJT_FILE)
out_folder_path = os.path.join(FOLDER, "histograms")

w = tf.summary.create_file_writer(out_folder_path)

with w.as_default():

    # Computing class distributions
    with open(class_file_path, "r") as f_in:

        epoch = 1
        class_iter = tqdm(f_in)
        for line in class_iter:
            line = line.replace("\n", "")
            distribution = line.split(" ")
            try:
                distribution.remove("")
            except(ValueError):
                pass
            distribution = numpy.array(distribution, dtype=numpy.double)

            tf.summary.histogram("class distributions", distribution, step=epoch)
            epoch += 1

            class_iter.set_description("Computing class distributions")

    # """
    # Computing commands distributions
    with open(cmds_file_path, "r") as f_in:

        epoch = 1
        cmds_iter = tqdm(f_in)
        for line in cmds_iter:
            line = line.replace("\n", "")
            distribution = line.split(" ")
            try:
                distribution.remove("")
            except(ValueError):
                pass
            distribution = numpy.array(distribution, dtype=numpy.double)

            tf.summary.histogram("commands distributions", distribution, step=epoch)
            epoch += 1

            cmds_iter.set_description("Computing commands distributions")
    #"""

    
    # Computing reject distributions
    with open(rjt_file_path, "r") as f_in:
        
        epoch = 1
        rjt_iter = tqdm(f_in)
        for line in rjt_iter:
            line = line.replace("\n", "")
            distribution = line.split(" ")
            try:
                distribution.remove("")
            except(ValueError):
                pass
            distribution = numpy.array(distribution, dtype=numpy.double)

            tf.summary.histogram("reject distributions", distribution, step=epoch)
            epoch += 1

            rjt_iter.set_description("Computing reject distributions")