import re
import numpy as np
from scipy import stats

REGEX = "<.*>"
SAMPLE_1_PATH = "/mnt/sdc1/sbini/command_interaction/training/testing/PEM.txt"
#SAMPLE_1_PATH = "/mnt/sdc1/sbini/command_interaction/training/testing/UniCL_PEM_v1.txt"
#SAMPLE_1_PATH = "/mnt/sdc1/sbini/command_interaction/training/testing/UniCL_PEM_v2.txt"
#SAMPLE_1_PATH = "/mnt/sdc1/sbini/command_interaction/training/testing/GaussCL_PEM_v1.txt"
#SAMPLE_1_PATH = "/mnt/sdc1/sbini/command_interaction/training/testing/GaussCL_PEM_v2.txt"

#SAMPLE_2_PATH = "/mnt/sdc1/sbini/command_interaction/training/testing/PEM.txt"
#SAMPLE_2_PATH = "/mnt/sdc1/sbini/command_interaction/training/testing/UniCL_PEM_v1.txt"
#SAMPLE_2_PATH = "/mnt/sdc1/sbini/command_interaction/training/testing/UniCL_PEM_v2.txt"
SAMPLE_2_PATH = "/mnt/sdc1/sbini/command_interaction/training/testing/GaussCL_PEM_v1.txt"
#SAMPLE_2_PATH = "/mnt/sdc1/sbini/command_interaction/training/testing/GaussCL_PEM_v2.txt"

sample_1 = list()
with open(SAMPLE_1_PATH, 'r') as fin:
    for line in fin:
        if '-' in line:
            break
        sample_1.append(float(re.findall(pattern=REGEX, string=line)[0][1:-1]))

sample_2 = list()
with open(SAMPLE_2_PATH, 'r') as fin:
    for line in fin:
        if '-' in line:
            break
        sample_2.append(float(re.findall(pattern=REGEX, string=line)[0][1:-1]))

sample1_bar, sample2_bar = np.mean(sample_1), np.mean(sample_2)
n1, n2 = len(sample_1), len(sample_2)
var_sample1, var_sample2= np.var(sample_1, ddof=1), np.var(sample_2, ddof=1)
# pooled sample variance
var = ( ((n1-1)*var_sample1) + ((n2-1)*var_sample2) ) / (n1+n2-2)
# standard error
std_error = np.sqrt(var * (1.0 / n1 + 1.0 / n2))

print("sample_1 mean:",np.round(sample1_bar,4))
print("sample_2 mean:",np.round(sample2_bar,4))
print("variance of sample_1:",np.round(var_sample1,4))
print("variance of sample_2:",np.round(var_sample2,4))
print("pooled sample variance:",var)
print("standard error:",std_error)

# calculate t statistics
t = abs(sample1_bar - sample2_bar) / std_error
print('t static:',t)
# two-tailed critical value at alpha = 0.05
t_c = stats.t.ppf(q=0.975, df=12)
print("Critical value for t two tailed:",t_c)
 
 
# one-tailed critical value at alpha = 0.05
t_c = stats.t.ppf(q=0.95, df=12)
print("Critical value for t one tailed:",t_c)
 
 
# get two-tailed p value
p_two = 2*(1-stats.t.cdf(x=t, df=12))
print("p-value for two tailed:",p_two)
 
# get one-tailed p value
p_one = 1-stats.t.cdf(x=t, df=12)
print("p-value for one tailed:",p_one)