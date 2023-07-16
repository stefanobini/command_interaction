import os
import pandas
import numpy as np
from scipy import stats


LANG = "ita"    # ["eng", "ita"]
HEADINGS = ["fold", "accuracy", "balanced_accuracy", "reject_accuracy"]

APPROACH_1 = "PEM"                  # ["PEM", "UniCL_PEM_v1", "UniCL_PEM_v2", "GaussCL_PEM_v1", "GaussCL_PEM_v2"]
MODEL_1 = "resnet8"                 # ["resnet8", "mobilenetv2", "conformer"]
APPROACH_2 = "UniCL_PEM_v2"         # ["PEM", "UniCL_PEM_v1", "UniCL_PEM_v2", "GaussCL_PEM_v1", "GaussCL_PEM_v2"]
MODEL_2 = "resnet8"                 # ["resnet8", "mobilenetv2", "conformer"]

SAMPLE_PATH_1 = os.path.join("lightning_logs", "SCR", LANG, MODEL_1, APPROACH_1, "test_results.csv")
SAMPLE_PATH_2 = os.path.join("lightning_logs", "SCR", LANG, MODEL_2, APPROACH_2, "test_results.csv")


def get_distribution(df:pandas.DataFrame, col:str):
    col = "accuracy" if "accuracy" in list(df.columns) else col     # for single task network, because has only one accuracy column
    l = list()
    for idx in df.index:
        l.append(df[col][idx])
    return l


df_1 = pandas.read_csv(SAMPLE_PATH_1, sep=',')
df_2 = pandas.read_csv(SAMPLE_PATH_2, sep=',')

col = "accuracy"

sample_1 = get_distribution(df=df_1, col=col)
sample_2 = get_distribution(df=df_2, col=col)

sample1_bar, sample2_bar = np.mean(sample_1), np.mean(sample_2)
n1, n2 = len(sample_1), len(sample_2)
var_sample1, var_sample2= np.var(sample_1, ddof=1), np.var(sample_2, ddof=1)
# pooled sample variance
var = ( ((n1-1)*var_sample1) + ((n2-1)*var_sample2) ) / (n1+n2-2)
# standard error
std_error = np.sqrt(var * (1.0 / n1 + 1.0 / n2))

print("{}-{} (mean-variance): {}{}{}".format(APPROACH_1, MODEL_1, np.round(sample1_bar,4), u"\u00B1", np.round(var_sample1,4)))
print("{}-{} (mean-variance): {}{}{}".format(APPROACH_2, MODEL_2, np.round(sample2_bar,4), u"\u00B1", np.round(var_sample2,4)))
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