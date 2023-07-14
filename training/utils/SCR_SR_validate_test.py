import os
import pandas
import numpy as np
from scipy import stats


LANG = "eng"    # ["eng", "ita"]
SINGLE_HEADINGS = ["fold", "accuracy"]
MULTI_HEADINGS = ["fold", "command_accuracy", "speaker_accuracy"]
TASK = "SI"

TASK_1 = "SI"                   # ["SCR", "SI", "SCR_SI"]
APPROACH_1 = "resnet8"     # ["resnet8", "HS_equal_weights", "SS_equal_weights", "HS_fixed_weights", "SS_fixed_weights", "HS_grad_norm", "SS_grad_norm"]
TASK_2 = "SCR_SI"                   # ["SCR", "SI", "SCR_SI"]
APPROACH_2 = "SS_fixed_weights"         # ["resnet8", "HS_equal_weights", "SS_equal_weights", "HS_fixed_weights", "SS_fixed_weights", "HS_grad_norm", "SS_grad_norm"]

SAMPLE_PATH_1 = os.path.join("lightning_logs", "MTL", LANG, TASK_1, APPROACH_1, "test_results.csv")
SAMPLE_PATH_2 = os.path.join("lightning_logs", "MTL", LANG, TASK_2, APPROACH_2, "test_results.csv")


def get_distribution(df:pandas.DataFrame, col:str):
    col = "accuracy" if "accuracy" in list(df.columns) else col     # for single task network, because has only one accuracy column
    l = list()
    for idx in df.index:
        l.append(df[col][idx])
    return l


df_1 = pandas.read_csv(SAMPLE_PATH_1, sep=',')
df_2 = pandas.read_csv(SAMPLE_PATH_2, sep=',')

col = "accuracy"
if TASK == "SCR":
    col = "command_accuracy"
elif TASK == "SI":
    col = "speaker_accuracy"

sample_1 = get_distribution(df=df_1, col=col)
sample_2 = get_distribution(df=df_2, col=col)

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