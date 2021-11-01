#!/usr/bin/python3

import sys
import os
import csv
import numpy as np
import matplotlib.pyplot as plt 
import pandas as pd

data_dir = "/home/yaphes/storage/humane_data/assessment/"
file_list = os.listdir(data_dir)

tlx_metric = ["Effort", "Performance", "Mental", "Temporal", "Frustration", "Physical"]


def retrieve_tlx_df(setting):
    m = {
        "Performance": [],
        "Mental Demand": [],
        "Temporal Demand": [],
        "Effort": [],
        "Frustration Level": [],
        "Physical Demand": [],
    }

    for file_str in file_list:
        if setting in file_str:
            with open(data_dir+file_str) as record:
                for line in record:
                    value =  line.split(",")
                    if value[0] != "Comment":
                        m[value[0]].append(float(value[1])*15)

    df = pd.DataFrame(m)
    df.rename(columns={"Mental Demand":"Mental", "Temporal Demand":"Temporal",
                        "Frustration Level":"Frustration", "Physical Demand":"Physical"}, inplace=True)
    return df
        
    

if __name__ == "__main__":
    setting_list = ["novel-button","novel-emotiv","steer-emotiv"]
    df = {}
    for setting in setting_list:
        new_label = setting.replace('-', '+')
        df[new_label] = retrieve_tlx_df(setting)

    # print (df['novel-button'])
    # print (df['novel-emotiv'])
    # print (df['steer-emotiv'])
    # means = df['novel-button'].mean(axis=0)
    # errors = df['novel-button'].sem(axis=0)  #use standard error instead of standard deviation 
    df_all = [df['steer+emotiv'],df['novel+emotiv'],df['novel+button']]
    df = pd.concat(df_all, keys=['steer+emotiv','novel+emotiv','novel+button'])

    df3 = df.groupby(level=0)

    means = df3.mean()
    errors = df3.sem()   #use standard error instead of standard deviation 
    means = means.reindex(['steer+emotiv','novel+emotiv','novel+button'])
    errors = errors.reindex(['steer+emotiv','novel+emotiv','novel+button'])
    # print(means)
    # print(errors)

    fig, ax = plt.subplots()
    bar_color = ['royalblue','red','orange']*6
    means.T.plot.bar(yerr=errors.T, ax=ax, grid=True, color=bar_color)  #plot transpose of means and errors
    ax.set_ylabel("NASA TLX score")
    plt.xticks(rotation=0)
    ax.autoscale()
    ax.legend(loc='upper left')
    plt.show()
    #plt.savefig(data_dir+"summary.pdf")
