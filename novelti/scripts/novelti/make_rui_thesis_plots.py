#!/usr/bin/env python
import os
import sys


from Bag2Plot import *
import matplotlib.gridspec as gridspec
import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd

from subprocess import call

DBASE = None;

LOCATIONS = {
    #name          x       y       yaw
    "rs1":        (19.5,   32.9,   0),
    "rd1":        (25.6,   14.4,   0.875),
    "rs2":        (14.7,   27.2,   0),
    "rd2":        (9.6,    8.0,    3.14),
    "rs3":        (39.4,   12.3,   0),
    "rd3":        (40.0,   11.2,   2.355)
}

BAG_STAMPS = {
    "yao": {
        "extredist_mx100": {
            "rs1tord1":          ["_2018-01-13-22-35-29","_2018-01-13-22-38-07","_2018-01-13-22-39-49","_2018-01-13-22-41-14","_2018-01-13-22-42-41"],
            "rs3tord3":          ["_2018-01-14-00-22-00","_2018-01-14-00-23-23","_2018-01-14-00-24-50","_2018-01-14-00-27-24"]
        },
        "extredist_mx91": {
            "rs1tord1":          ["_2018-01-13-22-54-13","_2018-01-13-22-57-51","_2018-01-13-23-00-41","_2018-01-13-23-03-20","_2018-01-13-23-05-17"],
            "rs3tord3":          ["_2018-01-14-14-15-31","_2018-01-14-14-17-22","_2018-01-14-14-19-00","_2018-01-14-14-20-53","_2018-01-14-14-22-44"]
        },
        "extredist_mx85": {
            "rs1tord1":          ["_2018-01-13-23-07-51","_2018-01-13-23-09-49","_2018-01-13-23-12-18","_2018-01-13-23-14-11","_2018-01-13-23-16-09"],
            "rs3tord3":          ["_2018-01-14-14-26-11","_2018-01-14-14-27-56","_2018-01-14-14-30-12","_2018-01-14-14-33-29","_2018-01-14-14-35-48"]
        },
        "altertile_mx100": {
            "rs1tord1":          ["_2018-01-13-23-19-58","_2018-01-13-23-22-01","_2018-01-13-23-23-20","_2018-01-13-23-24-35","_2018-01-13-23-25-51"],
            "rs3tord3":          ["_2018-01-14-14-45-52","_2018-01-14-14-47-14","_2018-01-14-14-48-33","_2018-01-14-14-49-46","_2018-01-14-14-50-55"]
        },
        "altertile_mx91": {
            "rs1tord1":          ["_2018-01-13-23-28-35","_2018-01-13-23-30-54","_2018-01-13-23-33-20","_2018-01-13-23-35-12","_2018-01-13-23-48-38"],
            "rs3tord3":          ["_2018-01-14-14-52-43","_2018-01-14-14-54-43","_2018-01-14-14-56-04","_2018-01-14-14-58-53","_2018-01-14-15-00-44"]
        },
        "altertile_mx85": {
            "rs1tord1":          ["_2018-01-13-23-51-20","_2018-01-13-23-53-14","_2018-01-13-23-55-10","_2018-01-13-23-59-45","_2018-01-14-00-01-38"],
            "rs3tord3":          ["_2018-01-14-15-02-57","_2018-01-14-15-05-13","_2018-01-14-15-07-32","_2018-01-14-15-09-15","_2018-01-14-15-12-05"]
        }
    },
    "rui": {
        "extredist_mx100": {
            "rs1tord1":          ["_2018-01-17-15-50-53","_2018-01-17-15-54-48","_2018-01-17-15-55-42","_2018-01-17-15-56-50","_2018-01-17-15-57-43"],
            "rs3tord3":          ["_2018-01-17-17-17-47","_2018-01-17-17-18-49","_2018-01-17-17-20-03","_2018-01-17-17-21-04","_2018-01-17-17-22-06"]
        },
        "extredist_mx91": {
            "rs1tord1":          ["_2018-01-17-16-18-23","_2018-01-17-16-20-08","_2018-01-17-16-23-08","_2018-01-17-16-24-24","_2018-01-17-16-38-46"],
            "rs3tord3":          ["_2018-01-17-17-25-20","_2018-01-17-17-27-29","_2018-01-17-17-29-13","_2018-01-17-17-30-32","_2018-01-17-17-31-54"]
        },
        "extredist_mx85": {
            "rs1tord1":          ["_2018-01-17-16-40-18","_2018-01-17-16-42-12","_2018-01-17-16-44-48","_2018-01-17-16-46-26","_2018-01-17-16-47-57"],
            "rs3tord3":          ["_2018-01-17-17-33-43","_2018-01-17-17-35-26","_2018-01-17-17-37-11","_2018-01-17-17-38-34","_2018-01-17-17-40-04"]
        },
        "altertile_mx100": {
            "rs1tord1":          ["_2018-01-17-16-50-22","_2018-01-17-16-51-40","_2018-01-17-16-52-39","_2018-01-17-16-53-37","_2018-01-17-16-54-38"],
            "rs3tord3":          ["_2018-01-17-20-31-24","_2018-01-17-20-34-27","_2018-01-17-20-35-35","_2018-01-17-20-36-32","_2018-01-17-20-37-38"]
        },
        "altertile_mx91": {
            "rs1tord1":          ["_2018-01-17-16-56-39","_2018-01-17-16-57-42","_2018-01-17-16-58-44","_2018-01-17-16-59-53","_2018-01-17-17-01-23"],
            "rs3tord3":          ["_2018-01-17-20-39-35","_2018-01-17-20-41-25","_2018-01-17-20-43-56","_2018-01-17-20-45-38","_2018-01-17-20-47-09"]
        },
        "altertile_mx85": {
            "rs1tord1":          ["_2018-01-17-17-03-10","_2018-01-17-17-04-39","_2018-01-17-17-05-44","_2018-01-17-17-07-10","_2018-01-17-17-08-28"],
            "rs3tord3":          ["_2018-01-17-20-50-34","_2018-01-17-20-52-14","_2018-01-17-20-55-52","_2018-01-17-20-58-22","_2018-01-17-21-00-14"]
        }
    },
    "mao": {
        "extredist_mx100": {
            "rs1tord1":          ["_2018-01-18-15-36-33","_2018-01-18-15-39-10","_2018-01-18-15-41-02","_2018-01-18-15-42-18","_2018-01-18-15-43-36"],
            "rs3tord3":          ["_2018-01-19-14-52-34","_2018-01-19-14-54-16","_2018-01-19-14-56-08","_2018-01-19-14-57-36","_2018-01-19-14-59-07"]
        },
        "extredist_mx91": {
            "rs1tord1":          ["_2018-01-18-15-45-29","_2018-01-18-15-49-00","_2018-01-18-15-50-46","_2018-01-18-15-52-40","_2018-01-18-15-54-03"],
            "rs3tord3":          ["_2018-01-19-15-01-49","_2018-01-19-15-04-37","_2018-01-19-15-06-07","_2018-01-19-15-08-57","_2018-01-19-15-10-19"]
        },
        "extredist_mx85": {
            "rs1tord1":          ["_2018-01-18-15-58-39","_2018-01-18-16-00-59","_2018-01-18-16-02-55","_2018-01-18-16-05-06","_2018-01-18-16-06-37"],
            "rs3tord3":          ["_2018-01-19-15-13-37","_2018-01-19-15-16-14","_2018-01-19-15-18-18","_2018-01-19-15-19-40","_2018-01-19-15-22-03"]
        },
        "altertile_mx100": {
            "rs1tord1":          ["_2018-01-18-16-12-50","_2018-01-18-16-14-23","_2018-01-18-16-15-49","_2018-01-18-16-17-34","_2018-01-18-16-19-06"],
            "rs3tord3":          ["_2018-01-19-15-29-41","_2018-01-19-15-31-28","_2018-01-19-15-32-34","_2018-01-19-15-33-50","_2018-01-19-15-35-04"]
        },
        "altertile_mx91": {
            "rs1tord1":          ["_2018-01-18-16-21-38","_2018-01-18-16-24-44","_2018-01-18-16-26-49","_2018-01-18-16-37-48","_2018-01-18-16-39-36"],
            "rs3tord3":          ["_2018-01-19-15-37-15","_2018-01-19-15-39-20","_2018-01-19-15-42-06","_2018-01-19-15-43-36","_2018-01-19-15-45-42"]
        },
        "altertile_mx85": {
            "rs1tord1":          ["_2018-01-18-16-42-00","_2018-01-18-16-45-54","_2018-01-18-16-48-29","_2018-01-18-16-50-46","_2018-01-18-16-53-02"],
            "rs3tord3":          ["_2018-01-19-15-51-32","_2018-01-19-15-54-01","_2018-01-19-15-56-13","_2018-01-19-16-00-41","_2018-01-19-16-02-05"]
        }
    },
    "jiahao": {
        "extredist_mx100": {
            "rs1tord1":          ["_2018-01-18-17-17-25","_2018-01-18-17-18-47","_2018-01-18-17-20-17","_2018-01-18-17-21-46","_2018-01-18-17-23-07"],
            "rs3tord3":          ["_2018-01-18-18-25-04","_2018-01-18-18-26-26","_2018-01-18-18-28-30","_2018-01-18-18-29-43","_2018-01-18-18-30-54"]
        },
        "extredist_mx91": {
            "rs1tord1":          ["_2018-01-18-17-25-36","_2018-01-18-17-27-42","_2018-01-18-17-29-57","_2018-01-18-17-31-26","_2018-01-18-17-33-29"],
            "rs3tord3":          ["_2018-01-18-18-32-37","_2018-01-18-18-33-55","_2018-01-18-18-35-20","_2018-01-18-18-37-07","_2018-01-18-18-39-06"]
        },
        "extredist_mx85": {
            "rs1tord1":          ["_2018-01-18-17-35-56","_2018-01-18-17-38-04","_2018-01-18-17-39-33","_2018-01-18-17-41-28","_2018-01-18-17-44-03"],
            "rs3tord3":          ["_2018-01-18-18-41-11","_2018-01-18-18-42-34","_2018-01-18-18-46-26","_2018-01-18-18-48-45","_2018-01-18-18-50-52"]
        },
        "altertile_mx100": {
            "rs1tord1":          ["_2018-01-18-17-46-42","_2018-01-18-17-48-02","_2018-01-18-17-49-13","_2018-01-18-17-50-43","_2018-01-18-17-51-54"],
            "rs3tord3":          ["_2018-01-19-13-59-19","_2018-01-19-14-00-48","_2018-01-19-14-01-54","_2018-01-19-14-03-02","_2018-01-19-14-04-07"]
        },
        "altertile_mx91": {
            "rs1tord1":          ["_2018-01-18-17-53-43","_2018-01-18-17-55-07","_2018-01-18-17-57-03","_2018-01-18-17-58-35","_2018-01-18-18-01-04"],
            "rs3tord3":          ["_2018-01-19-14-05-55","_2018-01-19-14-07-38","_2018-01-19-14-09-22","_2018-01-19-14-11-23"]
        },
        "altertile_mx85": {
            "rs1tord1":          ["_2018-01-18-18-03-28","_2018-01-18-18-05-33","_2018-01-18-18-08-12","_2018-01-18-18-11-17","_2018-01-18-18-13-50"],
            "rs3tord3":          ["_2018-01-19-14-13-34","_2018-01-19-14-16-02","_2018-01-19-14-18-27","_2018-01-19-14-19-43","_2018-01-19-14-21-36"]
        }
    },
    "moham": {
        "extredist_mx100": {
            "rs1tord1":          ["_2018-01-13-22-35-29","_2018-01-13-22-38-07","_2018-01-13-22-39-49","_2018-01-13-22-41-14","_2018-01-13-22-42-41"],
            "rs3tord3":          ["_2018-01-14-00-22-00","_2018-01-14-00-23-23","_2018-01-14-00-24-50","_2018-01-14-00-27-24"]
        },
        "extredist_mx91": {
            "rs1tord1":          ["_2018-01-13-22-54-13","_2018-01-13-22-57-51","_2018-01-13-23-00-41","_2018-01-13-23-03-20","_2018-01-13-23-05-17"],
            "rs3tord3":          ["_2018-01-14-14-15-31","_2018-01-14-14-17-22","_2018-01-14-14-19-00","_2018-01-14-14-20-53","_2018-01-14-14-22-44"]
        },
        "extredist_mx85": {
            "rs1tord1":          ["_2018-01-13-23-07-51","_2018-01-13-23-09-49","_2018-01-13-23-12-18","_2018-01-13-23-14-11","_2018-01-13-23-16-09"],
            "rs3tord3":          ["_2018-01-14-14-26-11","_2018-01-14-14-27-56","_2018-01-14-14-30-12","_2018-01-14-14-33-29","_2018-01-14-14-35-48"]
        },
        "altertile_mx100": {
            "rs1tord1":          ["_2018-01-13-23-19-58","_2018-01-13-23-22-01","_2018-01-13-23-23-20","_2018-01-13-23-24-35","_2018-01-13-23-25-51"],
            "rs3tord3":          ["_2018-01-14-14-45-52","_2018-01-14-14-47-14","_2018-01-14-14-48-33","_2018-01-14-14-49-46","_2018-01-14-14-50-55"]
        },
        "altertile_mx91": {
            "rs1tord1":          ["_2018-01-13-23-28-35","_2018-01-13-23-30-54","_2018-01-13-23-33-20","_2018-01-13-23-35-12","_2018-01-13-23-48-38"],
            "rs3tord3":          ["_2018-01-14-14-52-43","_2018-01-14-14-54-43","_2018-01-14-14-56-04","_2018-01-14-14-58-53","_2018-01-14-15-00-44"]
        },
        "altertile_mx85": {
            "rs1tord1":          ["_2018-01-13-23-51-20","_2018-01-13-23-53-14","_2018-01-13-23-55-10","_2018-01-13-23-59-45","_2018-01-14-00-01-38"],
            "rs3tord3":          ["_2018-01-14-15-02-57","_2018-01-14-15-05-13","_2018-01-14-15-07-32","_2018-01-14-15-09-15","_2018-01-14-15-12-05"]
        }
    },
    "test": {
        "extredist_mx100": {
            "rs1tord1":         ["_2018-01-29-16-04-12","_2018-01-29-16-05-23","_2018-01-29-16-06-09","_2018-01-29-16-06-54","_2018-01-29-16-07-38"]
        },
        "extredist_mx91": {
            "rs1tord1":         ["_2018-01-29-15-33-58","_2018-01-29-15-35-06","_2018-01-29-15-35-51","_2018-01-29-15-36-50","_2018-01-29-15-37-50"]
        },
        "extredist_mx85": {
            "rs1tord1":         ["_2018-01-29-15-39-07","_2018-01-29-15-40-16","_2018-01-29-15-41-35","_2018-01-29-15-42-27","_2018-01-29-15-43-15"]
        }
    },
    "extredist_mx100": {
        "rs1tord1":          ["_2018-01-17-15-50-53","_2018-01-17-15-54-48","_2018-01-17-15-55-42","_2018-01-17-15-56-50","_2018-01-17-15-57-43"],
        "rs3tord3":          ["_2018-01-17-17-17-47","_2018-01-17-17-18-49","_2018-01-17-17-20-03","_2018-01-17-17-21-04","_2018-01-17-17-22-06"]
    },
    "extredist_mx91": {
        "rs1tord1":          ["_2018-01-17-16-18-23","_2018-01-17-16-20-08","_2018-01-17-16-23-08","_2018-01-17-16-24-24","_2018-01-17-16-38-46"],
        "rs3tord3":          ["_2018-01-17-17-25-20","_2018-01-17-17-27-29","_2018-01-17-17-29-13","_2018-01-17-17-30-32","_2018-01-17-17-31-54"]
    },
    "extredist_mx85": {
        "rs1tord1":          ["_2018-01-17-16-40-18","_2018-01-17-16-42-12","_2018-01-17-16-44-48","_2018-01-17-16-46-26","_2018-01-17-16-47-57"],
        "rs3tord3":          ["_2018-01-17-17-33-43","_2018-01-17-17-35-26","_2018-01-17-17-37-11","_2018-01-17-17-38-34","_2018-01-17-17-40-04"]
    },
    "altertile_mx100": {
        "rs1tord1":          ["_2018-01-17-16-50-22","_2018-01-17-16-51-40","_2018-01-17-16-52-39","_2018-01-17-16-53-37","_2018-01-17-16-54-38"],
        "rs3tord3":          ["_2018-01-17-20-31-24","_2018-01-17-20-34-27","_2018-01-17-20-35-35","_2018-01-17-20-36-32","_2018-01-17-20-37-38"]
    },
    "altertile_mx91": {
        "rs1tord1":          ["_2018-01-17-16-56-39","_2018-01-17-16-57-42","_2018-01-17-16-58-44","_2018-01-17-16-59-53","_2018-01-17-17-01-23"],
        "rs3tord3":          ["_2018-01-17-20-39-35","_2018-01-17-20-41-25","_2018-01-17-20-43-56","_2018-01-17-20-45-38","_2018-01-17-20-47-09"]
    },
    "altertile_mx85": {
        "rs1tord1":          ["_2018-01-17-17-03-10","_2018-01-17-17-04-39","_2018-01-17-17-05-44","_2018-01-17-17-07-10","_2018-01-17-17-08-28"],
        "rs3tord3":          ["_2018-01-17-20-50-34","_2018-01-17-20-52-14","_2018-01-17-20-55-52","_2018-01-17-20-58-22","_2018-01-17-21-00-14"]
    },
    "change_mind_mx100": {
        "rs1tord1":         ["_2018-01-17-21-06-58","_2018-01-17-21-10-09","_2018-01-17-21-11-35","_2018-01-17-21-12-40","_2018-01-17-21-13-45"]
    },
    "change_mind_mx91": {
        "rs1tord1":         ["_2018-01-17-21-15-55","_2018-01-17-21-17-32","_2018-01-17-21-22-04","_2018-01-17-21-24-43","_2018-01-17-21-26-34"]
    },
    "orientation_opt_mx100": {
        "rs1tord1":          ["_2018-01-17-21-40-51","_2018-01-17-21-41-54","_2018-01-17-21-42-51","_2018-01-20-15-48-59"]
    },
    "orientation_opt_mx91": {
        "rs1tord1":          ["_2018-01-17-16-18-23","_2018-01-17-16-20-08","_2018-01-17-16-23-08","_2018-01-20-15-50-11"]
    },
    "orientation_opt_mx85": {
        "rs1tord1":          ["_2018-01-17-16-40-18","_2018-01-17-16-42-12","_2018-01-17-16-44-48","_2018-01-20-15-52-05"]
    },
    "orientation_still_mx100": {
        "rs1tord1":          ["_2018-01-17-21-48-13","_2018-01-17-21-49-10","_2018-01-17-21-50-14","_2018-01-20-15-53-15"]
    },
    "orientation_still_mx91": {
        "rs1tord1":          ["_2018-01-20-15-24-58","_2018-01-20-15-26-46","_2018-01-20-15-28-03","_2018-01-20-15-54-28"]
    },
    "orientation_still_mx85": {
        "rs1tord1":          ["_2018-01-20-15-29-47","_2018-01-20-15-31-18","_2018-01-20-15-33-13","_2018-01-20-15-56-22"]
    },
}


def autolabel(ax, rects):
    """
    Attach a text label above each bar displaying its height
    """
    for rect in rects:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width()/2., 1.05*height,
                '%.1f' % float(height),
                ha='center', va='bottom')

def plot_set_of_tries(ax_map, ax_dist, ax_entr, bag_stamps, colors):
    lines = []
    for k,stamp in enumerate(bag_stamps):
        bagRecord = DBASE.getRecord(stamp)
        pbr = PlottableBagRecord(bagRecord)
        lines.append(pbr.plotPathDistEntropy(ax_map, ax_dist, ax_entr, colors[k])[0])
    return lines

def plot_orientation_control_tries(ax_map, ax_angle, ax_entr, bag_stamps, colors):
    lines = []
    for k,stamp in enumerate(bag_stamps):
        bagRecord = DBASE.getRecord(stamp)
        pbr = PlottableBagRecord(bagRecord)
        lines.append(pbr.plotAngleDistEntropy(ax_map, ax_angle, ax_entr, colors[k], "try "+str(k+1))[0])
    return lines

def plot_subjects_data(ax_bar, method_k, data, yerr, colors, label):
    bar_width = 0.3
    ind = np.arange(len(data))+1
    bar = ax_bar.bar(ind+bar_width*method_k, data, yerr=yerr, width=bar_width, align='center', color=colors[0], label=label)
    ax_bar.legend()
    ax_bar.grid(True)
    ax_bar.autoscale(True)
    ax_bar.set_ylabel("Error(m)", fontsize=10)
    ax_bar.set_xlabel("Tries", fontsize=10)
    ax_bar.set_xticks(ind)
    autolabel(ax_bar, bar)
    
def generate_subject_data_list(subject, method_id, route_id, data_name):
    data = []
    for stamp in BAG_STAMPS[subject][method_id][route_id]:  
        bagRecord = DBASE.getRecord(stamp)
        data.append(bagRecord[data_name])
    return data

def plot_time_bar(axes, method_names, method_ids, method_color, route_names, route_ids, subjects, kind):
    bars = []
    for route_k,route_name in enumerate(route_names):
        route_id = route_ids[route_k]
        for method_k, method in enumerate(method_names):
            mean = []
            yerr = []
            method_id = method_ids[method_k]
            for subject in subjects:
                data = generate_subject_data_list(subject, method_id, route_id, kind)
                mean.append(np.mean(data))
                yerr.append(np.std(data))
            bs = plot_subjects_data(ax_bar= axes, 
                                    method_k= method_k, 
                                    data = mean,
                                    yerr = yerr,
                                    colors= method_color[method_k],
                                    label= method_id)
            if route_k==0:
                bars.append(bs)
    return bars
        
def plot_oc_section(fig, sub_grid, locations, method_names, method_ids, method_color_sets, route_names, route_ids, stamp_sets):
    lines = []
    num_routes = len(route_names)
    gs = gridspec.GridSpecFromSubplotSpec(2, num_routes+1, 
            subplot_spec=sub_grid,
            hspace=0.1) #left=0.02, right=0.98, bottom=0.04, top=0.96) #, wspace=0.05)
    ax_map = plt.Subplot(fig, gs[:,0])
    fig.add_subplot(ax_map)
    #plot orientation selection circles here rui
    map_stamp = stamp_sets[method_ids[0]][route_ids[0]][0]
    PlottableBagRecord(DBASE.getRecord(map_stamp)).plotMapInflated(ax_map)
    #draw labels for locations
    for k, loc in enumerate(locations):
        ax_map.annotate(loc,
            xy=LOCATIONS[loc][:2], 
            xycoords='data',
            xytext=(0, 25), 
            textcoords='offset points',
            backgroundcolor=(1.0, 1.0, 0.0, 0.7),
            arrowprops=dict(
                facecolor=(1.0, 1.0, 0.0, 0.7), 
                linewidth=0.1,
                shrink=0.05, 
                frac=0.5, 
                width=4, 
                headwidth=9 ), #http://matplotlib.org/users/annotations_intro.html
            horizontalalignment='center', 
            verticalalignment='bottom', 
            fontsize=16)
    bag_record = DBASE.getRecord(map_stamp)
    change_point = bag_record['first_opdf_pose']
    ax_map.annotate("position inferred",
            xy=change_point,
            xycoords='data',
            xytext=(20,0), 
            textcoords='offset points',
            arrowprops=dict(arrowstyle="->",
                            connectionstyle="arc3"),
            fontsize=10)
    #plot entropy and angular distance here
    for route_k,route_name in enumerate(route_names):
        route_id  = route_ids[route_k]
        ax_entr = plt.Subplot(fig, gs[0,route_k+1])
        ax_entr.set_title(method_names[0], fontsize=20)
        fig.add_subplot(ax_entr)
        ax_angle = plt.Subplot(fig, gs[1,route_k+1], sharex=ax_entr)
        fig.add_subplot(ax_angle)
        for method_k, method in enumerate(method_names):
            method_id = method_ids[method_k]
            stamp_list = stamp_sets[method_id][route_id]
            ls = plot_orientation_control_tries(ax_map, ax_angle, ax_entr, stamp_list, method_color_sets[method_k])
            if route_k==0:
                lines += ls
        ax_entr.legend()
        ax_angle.legend()
    return lines

#This function draws only one section of plot on a picture
def plot_on_1map(fig, sub_grid, locations, method_names, method_ids, method_color_sets, route_names, route_ids, stamp_sets):
    lines = []
    num_routes = len(route_names)
    gs = gridspec.GridSpecFromSubplotSpec(2, num_routes+1, 
            subplot_spec=sub_grid,
            hspace=0.1) #left=0.02, right=0.98, bottom=0.04, top=0.96) #, wspace=0.05)
    ax_map = plt.Subplot(fig, gs[:,0])
    fig.add_subplot(ax_map)
    map_stamp = stamp_sets[method_ids[0]][route_ids[0]][0]
    PlottableBagRecord(DBASE.getRecord(map_stamp)).plotMapInflated(ax_map)
    #draw labels for locations:
    for k, loc in enumerate(locations):
        ax_map.annotate(loc,
            xy=LOCATIONS[loc][:2], 
            xycoords='data',
            xytext=(0, 25), 
            textcoords='offset points',
            backgroundcolor=(1.0, 1.0, 0.0, 0.7),
            #linewidth=0.1,
            arrowprops=dict(
                facecolor=(1.0, 1.0, 0.0, 0.7), 
                linewidth=0.1,
                #edgecolor='none',
                #alpha=0.7, 
                shrink=0.05, 
                frac=0.5, 
                width=4, 
                headwidth=9 ), #http://matplotlib.org/users/annotations_intro.html
            horizontalalignment='center', 
            verticalalignment='bottom', 
            fontsize=16)
    labels = []
    handles=[]
    max_tries = 5
    for method_k,method_name in enumerate(method_names):
        for try_k in xrange(len(BAG_STAMPS[method_ids[method_k]][route_ids[0]])):
            handle = mlines.Line2D([], [], 
                color=method_color_sets[method_k][try_k],
                linewidth=2) #, label="%s, try %d"%(method_name,try_k))
            handles.append(handle)
            labels.append("%s, try %d"%(method_name,try_k+1))
    ax_map.legend( handles, labels, loc='upper center', bbox_to_anchor=[0.5,1.15], ncol=max_tries, fontsize = 7, columnspacing=0.1 )
    for route_k,route_name in enumerate(route_names):
        route_id  = route_ids[route_k]
        ax_entr = plt.Subplot(fig, gs[0,route_k+1])
        ax_entr.set_title(route_names[route_k], fontsize=20)
        fig.add_subplot(ax_entr)
        ax_dist = plt.Subplot(fig, gs[1,route_k+1], sharex=ax_entr)
        fig.add_subplot(ax_dist)
        for method_k, method in enumerate(method_names):
            method_id = method_ids[method_k]
            stamp_list = stamp_sets[method_id][route_id]
            ls = plot_set_of_tries(ax_map, ax_dist, ax_entr, stamp_list, method_color_sets[method_k])
            if route_k==0:
                lines += ls
    return lines

def plot_compare_2route_1map(title, method_names, method_ids, method_color_sets):
    fig = plt.figure(figsize=(2*8.5, 2*11.0), facecolor='white')
    outer_grid = gridspec.GridSpec(2, 1)
    outer_grid.update(left=0.02, right=0.98, bottom=0.04, top=0.9) #, wspace=0.05)
    lines = plot_on_1map(fig, outer_grid[0], 
                locations= ["rs1","rd1"],
                method_names=      method_names,
                method_ids=        method_ids,
                method_color_sets= method_color_sets,
                route_names= ["rs1->rd1"],
                route_ids=   ["rs1tord1",],
                stamp_sets=BAG_STAMPS)
    plot_on_1map(fig, outer_grid[1], 
                locations= ["rs3","rd3"],
                method_names=      method_names,
                method_ids=        method_ids,
                method_color_sets= method_color_sets,
                route_names= ["rs3->rd3"],
                route_ids=   ["rs3tord3"],
                stamp_sets=BAG_STAMPS)
    #plt.tight_layout()
    fig.autolayout = True
    
    # nplots = 2
    # labels = []
    # handles=[]
    # for try_k in xrange(nplots):
    #     for method_k,method_name in enumerate(method_names):
    #         print(len(BAG_STAMPS[method_ids[method_k]]['rs1tord1']))
    #         handle = mlines.Line2D([], [], 
    #             color=method_color_sets[method_k][try_k],
    #             linewidth=2) #, label="%s, try %d"%(method_name,try_k))
    #         handles.append(handle)
    #         labels.append("%s, try %d"%(method_name,try_k+1))
    # fig.legend( handles, labels, loc='upper left', bbox_to_anchor=[0.02,0.0,0.98,0.95], ncol=ntries )
    fig.suptitle(title, fontsize=20,horizontalalignment='center')
    return outer_grid

def plot_change_mind(title, method_names, method_ids, method_color_sets):
    fig = plt.figure(figsize=(2*8.5, 2*11.0), facecolor='white')
    #plt.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.1)
    outer_grid = gridspec.GridSpec(2, 1)
    outer_grid.update(left=0.02, right=0.98, bottom=0.04, top=0.9) #, wspace=0.05)
    lines =plot_on_1map(fig, outer_grid[0], 
                locations= ["rs1","rd1"],
                method_names=      [method_names[0]],
                method_ids=        [method_ids[0]],
                method_color_sets= method_color_sets,
                route_names= ["rs1->(rm1)->rd1"],
                route_ids=   ["rs1tord1"],
                stamp_sets=BAG_STAMPS)
    plot_on_1map(fig, outer_grid[1], 
                locations= ["rs1","rd1"],
                method_names=      [method_names[1]],
                method_ids=        [method_ids[1]],
                method_color_sets= method_color_sets,
                route_names= ["rs1->(rm1)->rd1"],
                route_ids=   ["rs1tord1"],
                stamp_sets=BAG_STAMPS)

    #plt.tight_layout()
    fig.autolayout = True
    fig.suptitle(title, fontsize=25,horizontalalignment='center')
    return outer_grid

def plot_time(title, method_names, method_ids, method_color_sets, kind):
    fig = plt.figure(figsize=(15, 20), facecolor='white')
    ax_bar_1 = fig.add_subplot(2,1,1)
    plot_time_bar(axes= ax_bar_1, 
                method_names= method_names, 
                method_ids= method_ids, 
                method_color= method_color_sets,
                route_names=['rs1->rd1'],
                route_ids=  ["rs1tord1"],
                subjects=['yao','rui','mao','jiahao'],
                kind=kind)
    ax_bar_1.set_title("route1",fontdict={'fontsize':15})
    ax_bar_2 = fig.add_subplot(2,1,2)
    plot_time_bar(axes= ax_bar_2, 
                method_names= method_names, 
                method_ids= method_ids, 
                method_color= method_color_sets,
                route_names=['rs3->rd3'],
                route_ids=  ["rs3tord3"],
                subjects=['yao','rui','mao','jiahao'],
                kind=kind)  
    ax_bar_2.set_title("route2",fontdict={'fontsize':15})
    fig.autolayout = True
    fig.suptitle(title, fontsize=25,horizontalalignment='center')

def plot_time_short(title, method_names, method_ids, method_color_sets, kind):
    fig = plt.figure(figsize=(15, 20), facecolor='white')
    ax_bar_1 = fig.add_subplot(1,1,1)
    plot_time_bar(axes= ax_bar_1, 
                method_names= method_names, 
                method_ids= method_ids, 
                method_color= method_color_sets,
                route_names=['rs1->rd1'],
                route_ids=  ["rs1tord1"],
                subjects=['test'],
                kind=kind)
    ax_bar_1.set_title("route1",fontdict={'fontsize':15})
    fig.autolayout = True
    fig.suptitle(title, fontsize=25,horizontalalignment='center')    


def plot_compare_orientation_strat(title, method_names, method_ids, method_color_sets):
    fig = plt.figure(figsize=(2*8.5, 2*11.0), facecolor='white')
    #plt.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.1)
    outer_grid = gridspec.GridSpec(3, 1)
    outer_grid.update(left=0.02, right=0.98, bottom=0.04, top=0.9) #, wspace=0.05)
    lines =plot_oc_section(fig, outer_grid[0], 
                locations= ["rs1","rd1"],
                method_names=      [method_names[0]],
                method_ids=        [method_ids[0]],
                method_color_sets= method_color_sets,
                route_names= ["rs1->rd1"],
                route_ids=   ["rs1tord1"],
                stamp_sets=BAG_STAMPS)
    lines =plot_oc_section(fig, outer_grid[1], 
                locations= ["rs1","rd1"],
                method_names=      [method_names[1]],
                method_ids=        [method_ids[1]],
                method_color_sets= method_color_sets,
                route_names= ["rs1->rd1"],
                route_ids=   ["rs1tord1"],
                stamp_sets=BAG_STAMPS)
    lines =plot_oc_section(fig, outer_grid[2], 
                locations= ["rs1","rd1"],
                method_names=      [method_names[2]],
                method_ids=        [method_ids[2]],
                method_color_sets= method_color_sets,
                route_names= ["rs1->rd1"],
                route_ids=   ["rs1tord1"],
                stamp_sets=BAG_STAMPS)            

    #This part only plot labels for tries
    fig.autolayout = True
    # ntries = 4
    # labels = []
    # handles=[]
    # for try_k in xrange(ntries):
    #     for method_k,method_name in enumerate(method_names):
    #         handle = mlines.Line2D([], [], 
    #             color=method_color_sets[method_k][try_k],
    #             linewidth=2) #, label="%s, try %d"%(method_name,try_k))
    #         handles.append(handle)
    #         labels.append("%s, try %d"%(method_name,try_k+1))
    # fig.legend( handles, labels, loc='upper left', bbox_to_anchor=[0.02,0.0,0.98,0.95], ncol=ntries )
    fig.suptitle(title, fontsize=25,horizontalalignment='center')
    return outer_grid

def plot_dst_distribution(title, axe, subjects, method_names, method_ids, route_names, route_ids):
    bars = []
    dst = []
    for route_k,route_name in enumerate(route_names):
        route_id = route_ids[route_k]
        for method_k, method in enumerate(method_names):
            method_id = method_ids[method_k]
            for subject in subjects:
                record = DBASE.getRecord(BAG_STAMPS[subject][method_id][route_id][0])
                goal = record['intended_dst']
                data = generate_subject_data_list(subject, method_id, route_id, 'actual_dst')
                for item in data:
                    diff = round(sqrt((goal[0]-item[0])**2 + (goal[1]-item[1])**2),1)  #round error to 1 digit
                    dst.append(diff)
            bs = plot_subjects_data(ax_bar= axe, 
                                    method_k= method_k, 
                                    data= dst,
                                    yerr= [0]*len(dst),
                                    colors= ["#9999FF"],
                                    label= method_id)
            if route_k==0:
                bars.append(bs)
    axe.set_title(method_names[0])
    
    return bars
    
def plot_dst_distribution_port(title, subjects, method_names, method_ids, route_names, route_ids, out_file):
    print "\n\n======================================================================="
    print title
    print "======================================================================="

    fig = plt.figure(figsize=(2*8.5, 2*11.0), facecolor='white')
    # outer_grid = gridspec.GridSpec(3, 1)
    # outer_grid.update(left=0.02, right=0.98, bottom=0.04, top=0.9) #, wspace=0.05)

    plot_dst_distribution(
        title=title,
        axe=fig.add_subplot(3,1,1),
        subjects=subjects,
        method_names=[method_names[0]],
        method_ids=[method_ids[0]],
        route_names=route_names,
        route_ids=route_ids
    )
    plot_dst_distribution(
        title=title,
        axe=fig.add_subplot(3,1,2),
        subjects=subjects,
        method_names=[method_names[1]],
        method_ids=[method_ids[1]],
        route_names=route_names,
        route_ids=route_ids
    )
    plot_dst_distribution(
        title=title,
        axe=fig.add_subplot(3,1,3),
        subjects=subjects,
        method_names=[method_names[2]],
        method_ids=[method_ids[2]],
        route_names=route_names,
        route_ids=route_ids
    )
    fig.autolayout = True
    fig.suptitle(title, fontsize=25,horizontalalignment='center')
    plt.savefig(out_file)

def plot_time_port(title, method_names, method_ids, kind, out_file):
    print "\n\n======================================================================="
    print title
    print "======================================================================="
    blue_set = ["#9999FF"]
    red_set = ["#FF9999"]
    green_set = ["#99FF99"]
    
    plot_time(
        title=title,
        method_names=      method_names,
        method_ids=        method_ids,
        method_color_sets= [blue_set, red_set, green_set],
        kind=              kind
    )
    plt.savefig(out_file)

def plot_time_short_port(title, method_names, method_ids, kind, out_file):
    print "\n\n======================================================================="
    print title
    print "======================================================================="
    blue_set = ["#9999FF"]
    red_set = ["#FF9999"]
    green_set = ["#99FF99"]
    
    plot_time_short(
        title=title,
        method_names=      method_names,
        method_ids=        method_ids,
        method_color_sets= [blue_set, red_set, green_set],
        kind=              kind
    )
    plt.savefig(out_file)

def plot_compare_2route_1map_port(title, method_names, method_ids, out_file):
    print "\n\n======================================================================="
    print title
    print "======================================================================="
    blue_set = ["#9999FF","#3333FF","#000099","#9999CC","#3333CC"]
    red_set  = ["#FF9999","#FF3333","#BB0000","#FF0000","#BB3333"]
    
    plot_compare_2route_1map(
        title=title,
        method_names=      method_names,
        method_ids=        method_ids,
        method_color_sets= [blue_set,     red_set]
    )
    plt.savefig(out_file)
    #call(["evince", out_file])
    #plt.show()

def plot_change_mind_port(title, method_names, method_ids, out_file):
    print "\n\n======================================================================="
    print title
    print "======================================================================="
    blue_set = ["#9999FF","#3333FF","#000099","#9999CC","#3333CC"]
    red_set = ["#FF9999"]

    color_set = [blue_set]
    
    plot_change_mind(
        title=title,
        method_names=      method_names,
        method_ids=        method_ids,
        method_color_sets= color_set
    )
    plt.savefig(out_file)
    #call(["evince", out_file])
    #plt.show()    

def plot_compare_orientation_strat_port(title, method_names, method_ids, out_file):
    print "\n\n======================================================================="
    print title
    print "======================================================================="
    blue_set = ["#9999FF","#3333FF","#000099","#9999CC"]

    color_set = [blue_set]

    plot_compare_orientation_strat(
        title=title,
        method_names=   method_names,
        method_ids=     method_ids,
        method_color_sets=color_set
    )
    plt.savefig(out_file)

def plots_for_thesis(output_file_prefix, plot_name, extension):
    if plot_name == "extredist_mx100_vs_mx91":
        plot_compare_2route_1map_port(
            title=          "Extredist mx100-vs-mx91 2 routes",
            method_names=   ["Extredist 100% HMI","Extredist 91% HMI"],
            method_ids=     ["extredist_mx100","extredist_mx91"],
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "extredist_mx91_vs_mx85":
        plot_compare_2route_1map_port(
            title=          "Extredist mx91-vs-mx85 2 routes",
            method_names=   ["Extredist 91% HMI","Extredist 85% HMI"],
            method_ids=     ["extredist_mx91","extredist_mx85"],
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "altertile_mx100_vs_mx91":
        plot_compare_2route_1map_port(
            title=          "Altertile mx100-vs-mx91 2 routes",
            method_names=   ["Altertile 100% HMI","Altertile 91% HMI"],
            method_ids=     ["altertile_mx100","altertile_mx91"],
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "altertile_mx91_vs_mx85":
        plot_compare_2route_1map_port(
            title=          "Altertile mx91-vs-mx85 2 routes",
            method_names=   ["Altertile 91% HMI","Altertile 85% HMI"],
            method_ids=     ["altertile_mx91","altertile_mx85"],
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "extredist_vs_altertile_mx100":
        plot_compare_2route_1map_port(
            title=          "mx100 extredist-vs-altertile 2 routes",
            method_names=   ["Extredist 100% HMI","Altertile 100% HMI"],
            method_ids=     ["extredist_mx100","altertile_mx100"],
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "extredist_vs_altertile_mx91":
        plot_compare_2route_1map_port(
            title=          "mx91 extredist-vs-altertile 2 routes",
            method_names=   ["Extredist 91% HMI","Altertile 91% HMI"],
            method_ids=     ["extredist_mx91","altertile_mx91"],
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "change-mind-different-mx":
        plot_change_mind_port(
            title=          "change-mind-different-mx",
            method_names=   ["100% HMI","91% HMI"],
            method_ids=     ["change_mind_mx100","change_mind_mx91"],
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "extredist_navigation_time":
        plot_time_port(
            title=          "human experiments extredist navigation time",
            method_names=   ["extredist_mx100", "extredist_mx91", "extredist_mx85"],
            method_ids=     ["extredist_mx100", "extredist_mx91", "extredist_mx85"],
            kind=           "navigation_time",  #navigation_time, rotation_time, overall_time
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "altertile_navigation_time":
        plot_time_port(
            title=          "human experiments altertile navigation time",
            method_names=   ["altertile_mx100", "altertile_mx91", "altertile_mx85"],
            method_ids=     ["altertile_mx100", "altertile_mx91", "altertile_mx85"],
            kind=           "navigation_time",  #navigation_time, rotation_time, overall_time
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "extredist_overall_time":
        plot_time_port(
            title=          "human experiments extredist overall time",
            method_names=   ["extredist_mx100", "extredist_mx91", "extredist_mx85"],
            method_ids=     ["extredist_mx100", "extredist_mx91", "extredist_mx85"],
            kind=           "overall_time",  #navigation_time, rotation_time, overall_time
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "extredist_rotation_time":
            plot_time_port(
            title=          "human experiments rotation time 1",
            method_names=   ["extredist_mx100", "extredist_mx91", "extredist_mx85"],
            method_ids=     ["extredist_mx100", "extredist_mx91", "extredist_mx85"],
            kind=           "rotation_time",  #navigation_time, rotation_time, overall_time
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "altertile_rotation_time":
            plot_time_port(
            title=          "human experiments rotation time 2",
            method_names=   ["altertile_mx100", "altertile_mx91", "altertile_mx85"],
            method_ids=     ["altertile_mx100", "altertile_mx91", "altertile_mx85"],
            kind=           "rotation_time",  #navigation_time, rotation_time, overall_time
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "short_period_overall_time":
        plot_time_short_port(
            title=          "extredist short delay overall time",
            method_names=   ["extredist_mx100", "extredist_mx91", "extredist_mx85"],
            method_ids=     ["extredist_mx100", "extredist_mx91", "extredist_mx85"],
            kind=           "overall_time",  #navigation_time, rotation_time, overall_time
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "compare_orientation_strat":
        plot_compare_orientation_strat_port(
            title= "Optimal Intermediate Orientation Selector",
            method_names= ["orientation_opt_mx100", "orientation_opt_mx91", "orientation_opt_mx85"],
            method_ids=   ["orientation_opt_mx100","orientation_opt_mx91","orientation_opt_mx85"],
            out_file=       output_file_prefix + plot_name + extension
        )
    elif plot_name == "dst_distribution_extredist":
        plot_dst_distribution_port(
            title=        "Navigation Accuracy (Extredist)",
            subjects=     ['yao','rui','mao','jiahao'],
            method_names= ["Confusion Matrix mx100","Confusion Matrix mx91","Confusion Matrix mx85"],
            method_ids=   ['extredist_mx100','extredist_mx91','extredist_mx85'],
            route_names=  ['rs3 to rd3'],
            route_ids=    ['rs3tord3'],
            out_file=      output_file_prefix + plot_name+ extension
        )
    elif plot_name == "dst_distribution_altertile":
        plot_dst_distribution_port(
            title=        "Navigation Accuracy (Altertile)",
            subjects=     ['yao','rui','mao','jiahao'],
            method_names= ["Confusion Matrix mx100","Confusion Matrix mx91","Confusion Matrix mx85"],
            method_ids=   ['altertile_mx100','altertile_mx91','altertile_mx85'],
            route_names=  ['rs3 to rd3'],
            route_ids=    ['rs3tord3'],
            out_file=      output_file_prefix + plot_name+ extension
        )
    else:
        return False

    return True

if __name__=="__main__":
    bagDir = "/home/yaphes/bag/exp/"
    cacheDir = "/home/yaphes/data_cache"
    cwaveToolPath = "/home/yaphes/novelti_ws/devel/lib/novelti/cwave_cmdline"
    outputDir = "/home/yaphes/plot"
    outputFile ="rui_thesis_extredist_rotation_time.png"
  
    DBASE = RecordByStamp(bagDir, cacheDir, cwaveToolPath)
    
    thesis_prefix = "rui_thesis_"
    if outputFile.startswith(thesis_prefix):
        plot_name = outputFile[len(thesis_prefix):-4]
        extension = outputFile[-4:]
        if not plots_for_thesis(outputDir+"/"+thesis_prefix,  plot_name, extension):
            print "Unknown plot name for thesis plot: '%s'" % plot_name
            exit(1)