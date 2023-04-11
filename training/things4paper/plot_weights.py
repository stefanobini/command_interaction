import pandas
import matplotlib.pyplot as plt


IN_HEADINGS = ["Wall time", "Step", "Value"]
OUT_HEADINGS = ["Epoch", "HS_SCR", "HS_SR", "SS_SCR", "SS_SR"]
HS_SCR_CSV = "HS_grad_norm_SCR.csv"
HS_SR_CSV = "HS_grad_norm_SR.csv"
SS_SCR_CSV = "SS_grad_norm_SCR.csv"
SS_SR_CSV = "SS_grad_norm_SR.csv"


hs_scr_df = pandas.read_csv(HS_SCR_CSV, sep=',')
hs_sr_df = pandas.read_csv(HS_SR_CSV, sep=',')
ss_scr_df = pandas.read_csv(SS_SCR_CSV, sep=',')
ss_sr_df = pandas.read_csv(SS_SR_CSV, sep=',')

df = pandas.DataFrame(columns=OUT_HEADINGS)
lenght = min(len(hs_scr_df), len(hs_sr_df), len(ss_scr_df), len(ss_sr_df))

df["Epoch"] = [i+1 for i in range(lenght)]
df["HS_SCR"] = hs_scr_df["Value"][:lenght]
df["HS_SR"] = hs_sr_df["Value"][:lenght]
df["SS_SCR"] = ss_scr_df["Value"][:lenght]
df["SS_SR"] = ss_sr_df["Value"][:lenght]

plt.plot(df["Epoch"], df["HS_SCR"], label="HS_SCR", color="orange", linestyle="--")
plt.plot(df["Epoch"], df["HS_SR"], label="HS_SR", color="orange", linestyle=":")
plt.plot(df["Epoch"], df["SS_SCR"], label="SS_SCR", color="blue", linestyle="--")
plt.plot(df["Epoch"], df["SS_SR"], label="SS_SR", color="blue", linestyle=":")

plt.grid(linestyle = '-')
plt.legend()
# plt.show()
plt.savefig("gradNorm_weights.png")