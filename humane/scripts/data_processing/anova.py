import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import scipy.stats as stats
from bioinfokit.analys import stat
import statsmodels
import numpy as np

df = pd.read_csv('/home/yaphes/humane_data/organized/database.csv')

### path a2b:  group_ne: novel+emotiv, group_nb: novel+button, group_se: steer+emotiv
# a2b_group_ne = df[(df['Control Methods']=='novel+emotiv') & (df['Path']=='C to A')]['Total Commands']
# a2b_group_nb = df[(df['Control Methods']=='novel+button') & (df['Path']=='C to A')]['Total Commands']
# a2b_group_se = df[(df['Control Methods']=='steer+emotiv') & (df['Path']=='C to A')]['Total Commands']
# fvalue, pvalue = stats.f_oneway(a2b_group_nb, a2b_group_ne, a2b_group_se)
# print (fvalue, pvalue)

# print(df.query('control_id == "novel-emotiv" & path=="a2b"')['total_command'])
# res = stat()
# res.tukey_hsd(df=df[df['Path']=='C to A'], res_var='Total Commands', xfac_var='Control Methods', anova_model="'Total Commands' ~ C('Control Methods')")
# res.tukey_summary


df_a2b = df[df['Path']=='A to B']
df_b2c = df[df['Path']=='B to C']
df_c2a = df[df['Path']=='C to A']

def plot_comparison(metric, delta):
    mc_a2b = statsmodels.stats.multicomp.MultiComparison(df_a2b[metric], df_a2b['Control Methods'], group_order=['steer+emotiv', 'novel+emotiv', 'novel+button'])
    mc_a2b_hsd = mc_a2b.tukeyhsd()
    mc_b2c = statsmodels.stats.multicomp.MultiComparison(df_b2c[metric], df_b2c['Control Methods'], group_order=['steer+emotiv', 'novel+emotiv', 'novel+button'])
    mc_b2c_hsd = mc_b2c.tukeyhsd()
    mc_c2a = statsmodels.stats.multicomp.MultiComparison(df_c2a[metric], df_c2a['Control Methods'], group_order=['steer+emotiv', 'novel+emotiv', 'novel+button'])
    mc_c2a_hsd = mc_c2a.tukeyhsd()
    p_list = np.concatenate((mc_a2b_hsd.pvalues, mc_b2c_hsd.pvalues, mc_c2a_hsd.pvalues)) 
    print(p_list)

    box_width = 0.8
    ax = sns.boxplot(x='Path', y=metric, order=["A to B", "B to C", "C to A"], hue='Control Methods', data=df, width=box_width)
    x_centers = [0-box_width/3, 0, box_width/3, 1-box_width/3, 1, 1+box_width/3, 2-box_width/3, 2, 2+box_width/3]

    for i, p in enumerate(p_list):
        if (p < 0.05):
            text = "p<0.05"
            if (p < 0.01):
                text = "p<0.01"
            if (i < 3):
                y = df_a2b[metric].max() + delta
            elif (i < 6):
                y = df_b2c[metric].max() + delta
            else:
                y = df_c2a[metric].max() + delta
            col = 'k'
            h = delta
            if (i % 3 == 0):
                x1 = x_centers[i]
                x2 = x_centers[i+1]
                plt.plot([x1, x1, x2, x2],[y, y+h, y+h, y], lw=1.5, c=col)
                plt.text((x1+x2)*0.5, y+h, text, ha="center", va="bottom", color=col)
            elif (i % 3 == 1):
                x1 = x_centers[i-1]
                x2 = x_centers[i+1]
                y = y + 3*delta
                plt.plot([x1, x1, x2, x2],[y, y+h, y+h, y], lw=1.5, c=col)
                plt.text((x1+x2)*0.5, y+h, text, ha="center", va="bottom", color=col)
            elif (i % 3 == 2):
                x1 = x_centers[i-1]
                x2 = x_centers[i]
                y = y + 2*delta
                plt.plot([x1, x1, x2, x2],[y, y+h, y+h, y], lw=1.5, c=col)
                plt.text((x1+x2)*0.5, y+h, text, ha="center", va="bottom", color=col)
    
    print("Plotting "+metric)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # plot_comparison("Arrival Position Error (m)", 0.02)
    # plot_comparison("Arrival Orientation Error (rad)", 0.03)
    # plot_comparison("Navigation Time (s)", 15)
    plot_comparison("Total Commands", 2.5)



