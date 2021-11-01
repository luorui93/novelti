from math import sin,cos

import matplotlib.pyplot as plt
import numpy as np
from MapTools import GridMap
import cv2


def cross_comparison_bar(ax, mean_dict, std_dict):
    '''
    ax: The axes instance to be plotted on
    mean_dict: dictionary contains mean data for different groups
    std_dict:  dictionary contains std data for different groups
    '''
    assert len(mean_dict) == len(
        std_dict), 'Length of Means and Stds must be equal'
    bar_width = 0.06
    count = 0
    color_list = ['b', 'r', 'y', 'g', 'c']
    error_config = {'ecolor': '0.3'}

    for group_name, group_data in mean_dict.items():
        index = np.arange(len(group_data))*0.5
        ax.bar(index + count*bar_width*1.8, group_data, bar_width,
               alpha=1, color=color_list[count],
               yerr=std_dict[group_name], error_kw=error_config,
               label=group_name.replace('-','+'))
        count += 1

    for p in ax.patches:
        ax.annotate("%.2f" % p.get_height(), (p.get_x() + p.get_width() / 2 -0.035, p.get_height()), ha='center', va='center', xytext=(0, 10), fontsize=6, textcoords='offset points', rotation=90)
    ax.set_xticks(index + bar_width)
    return ax


def plotMap(data, axes):
    img = data.shared_data['exp_map']['map']
    #print (img.dtype)
    kernel = np.ones((7,7),np.uint8)
    erosion = cv2.erode(img.astype(np.uint8),kernel,iterations = 1)
    axes.imshow( erosion,
                 origin="lower",
                 cmap=plt.cm.gray,
                 vmin=0, vmax=1, norm=None,
                 extent=(0, data.shared_data['exp_map']['width'], 0, data.shared_data['exp_map']['height']))

def plotMapInflated(data, axes):
    axes.imshow(data.shared_data['exp_map']['map_inflated'],#[200:800,0:600],
                    origin="lower",
                    alpha=0.3,
                    cmap=plt.cm.gray,
                    vmin=0, vmax=1, norm=None,
                    extent=(0, data.shared_data['exp_map']['width'], 0, data.shared_data['exp_map']['height']))

def drawArrow(axes, x, y, a, color):
    arr_length = 0.4
    head_length = 0.18
    head_width = 0.18
    axes.arrow(x, y, arr_length*cos(a), arr_length*sin(a),
                length_includes_head=True,
                head_width=head_width,
                head_length=head_length,
                linewidth=0.3,
                # joinstyle='miter', #['miter' | 'round' | 'bevel']
                # capstyle='butt', #['butt' | 'round' | 'projecting']   http://stackoverflow.com/a/10297860/5787022
                fc=color,
                ec=color)

def plotEntropy(data, axes, color, label):
    #info("    Drawing entropy plot")
    l = axes.plot(data['entropy']['t'], data['entropy']['v'],
                    color=color,
                    linestyle='-',
                    label=label
                    )
    axes.grid(False)
    axes.autoscale(True)
    #axes.set_title("PDF entropy evolution over time", y=1.00)
    axes.set_ylabel("position entropy (bits)", fontsize=10)
    axes.set_xlabel("time(s)", fontsize=10)
    return l

def plotAngEntropy(data, axes, color, label):
    l = axes.plot(data['ang_entropy']['t'], data['ang_entropy']['v'],
                    color=color,
                    linestyle='-',
                    label=label,
                    )
    axes.autoscale(True)
    axes.set_ylabel("orientation entropy (bits)", fontsize=10)
    axes.set_xlabel("time(s)", fontsize=10)
    axes.grid(False)
    return l

def plotDistance(data, axes, color, label):
    #info("    Drawing distance plot")
    l = axes.plot(data['dist']['t'], data['dist']['v'],
                    color=color,
                    linestyle='-',
                    label=label,
                    )
    axes.autoscale(True)
    #axes.set_title("Distance to destination over time", y=1.00)
    axes.set_ylabel("position distance(m)", fontsize=10)
    axes.set_xlabel("time(s)", fontsize=10)
    axes.grid(False)
    return l

#Need to investigate later why ang_dist become a tuple
def plotAngDistance(data, axes, color, label):
    l = axes.plot(data['ang_dist'][0]['t'], data['ang_dist'][0]['v'],
                    color=color,
                    linestyle='-',
                    label=label,
                    )
    axes.grid(False)
    axes.autoscale(True)
    axes.set_ylabel("angular distance(rad)", fontsize=10)
    axes.set_xlabel("time(s)", fontsize=10)
    return l

def plotPath(data, axes, color):
    for k, t in enumerate(data['path']['t']):
        drawArrow(axes, data['path']['x'][k], data['path']
                        ['y'][k], data['path']['a'][k], color)

def plotPathDistEntropy(self, ax_map, ax_dist, ax_entr, color):
    self.plotPath(ax_map, color)
    line_dist = self.plotDistance(ax_dist, color)
    line_entr = self.plotEntropy(ax_entr, color)
    return (line_dist, line_entr)

def plotAngleDistEntropy(self, ax_map, ax_ang, ax_entr, color, label):
    # Plot orientation panel here
    self.plotPath(ax_map, color)
    angle_dist = self.plotAngDistance(ax_ang, color, label)
    angle_entr = self.plotAngEntropy(ax_entr, color, label)
    return (angle_dist, angle_entr)
