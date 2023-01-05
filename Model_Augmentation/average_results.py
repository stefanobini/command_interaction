import csv
import os
import numpy as np


FOLDER      = "/mnt/sdb1/sbini/Speech-Command_Interaction/Model_Augmentation/nemo_experiments/MatchboxNet-3x2x64/2022-12-12_15-30-44"
IN_FILE     = os.path.join(FOLDER, "res/stats.csv")
OUT_FILE    = os.path.join(FOLDER, "summary.txt")
SEEDs       = 10
SNRs        = range(-10, 25, 5)

res = dict()
acc_list, bal_acc_list, rjt_acc_list = np.zeros(shape=(len(SNRs), SEEDs)), np.zeros(shape=(len(SNRs), SEEDs)), np.zeros(shape=(len(SNRs), SEEDs))
snr_idx, seed_idx = 0, 0

with open(IN_FILE, "r") as f_in:
    csvreader = csv.reader(f_in)
    header = next(csvreader)

    for row in csvreader:
        seed, snr, acc, bal_acc, rjt_acc = float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4])
        
        acc_list[int(snr_idx/SEEDs), seed_idx%SEEDs] = acc
        bal_acc_list[int(snr_idx/SEEDs), seed_idx%SEEDs] = bal_acc
        rjt_acc_list[int(snr_idx/SEEDs), seed_idx%SEEDs] = rjt_acc

        snr_idx += 1
        seed_idx += 1
    
acc = np.mean(acc_list, axis=1)
var_acc = np.var(acc_list,axis=1)
bal_acc = np.mean(bal_acc_list, axis=1)
var_bal_acc = np.var(bal_acc_list,axis=1)
rjt_acc = np.mean(rjt_acc_list, axis=1)
var_rjt_acc = np.var(rjt_acc_list,axis=1)

with open(OUT_FILE, "w")as f_out:
    f_out.write("********************************\n")
    f_out.write("*********** Accuracy ***********\n")
    f_out.write("********************************\n")
    for i in range(0, len(SNRs)):
        f_out.write("* SNR {}dB\t: {:.3f} +- {:.5f} *\n".format(SNRs[i], acc[i], var_acc[i]))
    f_out.write("* Total \t: {:.3f} +- {:.5f} *\n".format(np.mean(acc), np.mean(var_acc)))
    f_out.write("********************************\n\n")

    f_out.write("********************************\n")
    f_out.write("****** Balanced accuracy *******\n")
    f_out.write("********************************\n")
    for i in range(0, len(SNRs)):
        f_out.write("* SNR {}dB\t: {:.3f} +- {:.5f} *\n".format(SNRs[i], bal_acc[i], var_bal_acc[i]))
    f_out.write("* Total \t: {:.3f} +- {:.5f} *\n".format(np.mean(bal_acc), np.mean(var_bal_acc)))
    f_out.write("********************************\n\n")

    f_out.write("********************************\n")
    f_out.write("******* Reject accuracy ********\n")
    f_out.write("********************************\n")
    for i in range(0, len(SNRs)):
        f_out.write("* SNR {}dB\t: {:.3f} +- {:.5f} *\n".format(SNRs[i], rjt_acc[i], var_rjt_acc[i]))
    f_out.write("* Total \t: {:.3f} +- {:.5f} *\n".format(np.mean(rjt_acc), np.mean(var_rjt_acc)))
    f_out.write("********************************")