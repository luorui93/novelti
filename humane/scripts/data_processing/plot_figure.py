#!/usr/bin/python
from __future__ import print_function
from data_process import DataBase
from collections import defaultdict
import bagplot
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import logging
import os
import pickle
import numpy as np

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
cache_path ='/home/yaphes/humane_data/organized/database.pickle'

BLUE = ["#3333FF","#9999FF","#000099","#9999CC","#3333CC"]
RED  = ["#FF3333","#FF9999","#BB0000","#FF0000","#BB3333"]

class CrossAnalysis:
    def __init__(self, database, xlabels=['a2b','b2c','c2a']):
        self.database = database
        self.xlabels = xlabels
    def __get_bar_group_data(self, group, metric):
        '''
        group: control_method, corresponds to different color groups
        metric: data to be plotted
        group_data = [[subgroup1],[subgroup2],...]
        '''
        group_mean = [[] for i in range(len(self.xlabels))]
        group_std =  [[] for i in range(len(self.xlabels))]
        logger.debug('----Retrieving {0} data from {1}-----'.format(metric, group))
        #In our case, key is path name, 
        for key, subgroup in self.database.data[group].items():
            subgroup_data = []
            for entry in subgroup:
                try:
                    subgroup_data.append(entry[metric])
                except KeyError:
                    logger.debug('{0} information is not in entry {1}'.format(metric, entry['file_id']))
            logger.debug('Retrieved subgroup data {0} for {1}-{2}'.format(subgroup_data,group,key))
            logger.debug("Calculating mean and standard deviation")
            mean = np.mean(subgroup_data)
            std  = np.std(subgroup_data)
            #this step is to ensure data is stored in designed order
            group_mean[self.xlabels.index(key)] = mean
            group_std[self.xlabels.index(key)] = std

        return group_mean, group_std
    def collect_bar_plot_data(self,group_list,metric):
        '''
        group_list:  list containing all groups/control_method
        metric: data to be plotted
        '''
        mean_dict = {}
        std_dict = {}
        for group in group_list:
            mean_dict[group], std_dict[group] = self.__get_bar_group_data(group, metric)
    
        return mean_dict, std_dict
    
    def collect_workload_value(self,group_list):
        '''
        group_list:  list containing all groups/control_method
        '''
        mean_dict = {}
        std_dict = {}
        for group in group_list:
            mean_dict[group], std_dict[group] = self.__get_bar_group_data(group, 'workload')

        return mean_dict, std_dict

class FigureLayout:
    def __init__(self, figure_type):
        '''
        figure_type can be:
        left_one_right_two,

        '''
        getattr(self, figure_type, lambda: logger.error("Invalid figure type"))()

    def left_one_right_two_a2b(self):
        self.fig = plt.figure(figsize=(10,4))
        gs = gridspec.GridSpec(2, 5, figure=self.fig)
        self.ax_left = self.fig.add_subplot(gs[:,0:2])
        self.ax_left.set_xlim(5,30)
        self.ax_left.set_ylim(65,80)
        self.ax_left.set_title('Navigation route')
        self.ax_right_up = self.fig.add_subplot(gs[0,2:5])
        self.ax_right_up.set_title('Distance to goal')
        self.ax_right_down = self.fig.add_subplot(gs[1,2:5])
        self.ax_right_down.set_title('Entropy')
        #plt.show()
    
        return self
    def left_one_right_two_b2c(self):
        self.fig = plt.figure(figsize=(10,4))
        gs = gridspec.GridSpec(2, 5, figure=self.fig)
        self.ax_left = self.fig.add_subplot(gs[:,0:2])
        self.ax_left.set_xlim(20,50)
        self.ax_left.set_ylim(50,80)
        self.ax_left.set_title('Navigation route')
        self.ax_right_up = self.fig.add_subplot(gs[0,2:5])
        self.ax_right_up.set_title('Distance to goal')
        self.ax_right_down = self.fig.add_subplot(gs[1,2:5])
        self.ax_right_down.set_title('Entropy')
    
        #plt.show()
    
        return self

    def left_one_right_two_c2a(self):
        self.fig = plt.figure(figsize=(10,4))
        gs = gridspec.GridSpec(2, 5, figure=self.fig)
        self.ax_left = self.fig.add_subplot(gs[:,0:2])
        self.ax_left.set_xlim(5,50)
        self.ax_left.set_ylim(50,75)
        self.ax_left.set_title('Navigation route')
        self.ax_right_up = self.fig.add_subplot(gs[0,2:5])
        self.ax_right_up.set_title('Distance to goal')
        self.ax_right_down = self.fig.add_subplot(gs[1,2:5])
        self.ax_right_down.set_title('Entropy')
        #plt.show()
    
        return self        

    def left_two_right_two(self):
        self.fig = plt.figure(figsize=(8,6))
        gs = gridspec.GridSpec(3, 4, figure=self.fig)
        self.ax_left_up = self.fig.add_subplot(gs[0:2,0:2])
        self.ax_left_up.set_xlim(0,50)
        self.ax_left_up.set_ylim(30,80)
        self.ax_left_up.set_title("Navigation route")
        self.ax_left_down = self.fig.add_subplot(gs[2:3,0:2])
        self.ax_left_down.set_title("Distance to goal")
        self.ax_right_up = self.fig.add_subplot(gs[0:2,2:4])
        self.ax_right_up.set_xlim(0,50)
        self.ax_right_up.set_ylim(30,80)
        self.ax_right_up.set_title("Navigation route")
        self.ax_right_down = self.fig.add_subplot(gs[2:3,2:4])
        self.ax_right_down.set_title("Distance to goal")

    def two_in_one_column(self):
        self.fig = plt.figure(figsize=(6,6))
        gs = gridspec.GridSpec(3, 3, figure=self.fig)
        self.ax_left_up = self.fig.add_subplot(gs[0:2,0:3])
        self.ax_left_up.set_xlim(5,50)
        self.ax_left_up.set_ylim(50,75)
        self.ax_left_up.set_title("Navigation route")
        self.ax_left_down = self.fig.add_subplot(gs[2:3,0:3])
        self.ax_left_down.set_title("Distance to goal")

    def multiple_in_one_row(self, num=5):
        self.fig = plt.figure(figsize=(num*2.5,2.5))
        gs = gridspec.GridSpec(1,num,figure=self.fig)
        gs.update(wspace=0.4, left=0.14, right=0.98, bottom=0.2)
        self.ax_list = []
        for i in range(num):
            ax = self.fig.add_subplot(gs[0,i])
            self.ax_list.append(ax)

def plot_cross_comparison_bar(ax, metric, xlabel, ylabel, title, save_path=None):
    paths = ['a2b','b2c','c2a']
    ca = CrossAnalysis(database,paths)
    means,stds = ca.collect_bar_plot_data(['novel-emotiv','novel-button','steer-emotiv'], metric)
    logger.info("mean of metric {0}:{1}".format(metric,means))
    logger.info("std of metric {0}:{1}".format(metric,stds))
    ax = bagplot.cross_comparison_bar(ax, means, stds)

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel,fontsize=8)
    #ax.set_title(title,fontsize=8)
    ax.set_xticklabels(['route1','route2','route3'])
    ax.tick_params(labelsize=6)
    ax.grid(axis='y',alpha=0.6)

   
    ax.spines['top'].set_visible(False)
    #ax.legend()

def plot_path_distance_entropy(database, entry_list, fig, color_list, save_path=None):
    #plot map
    bagplot.plotMap(database,fig.ax_left)
    bagplot.plotMapInflated(database,fig.ax_left)
    #plot path
    twin_ax_up = fig.ax_right_up.twinx()
    twin_ax_down = fig.ax_right_down.twinx()
    for index, entry in enumerate(entry_list):
        bagplot.plotPath(entry,fig.ax_left,color_list[index][0])
        #plot distance
        l1 = bagplot.plotDistance(entry,fig.ax_right_up,color_list[index][0],'position distance')
 
        l2 = bagplot.plotAngDistance(entry,twin_ax_up,color_list[index][1],'angular distance')
        #plot entropy
        l3 = bagplot.plotEntropy(entry,fig.ax_right_down,color_list[index][0],'position entropy')

        l4 = bagplot.plotAngEntropy(entry,twin_ax_down,color_list[index][1],'angular entropy')

    #create labels 
    # lu = l1 + l2
    # labu = [l.get_label() for l in lu]
    # fig.ax_right_up.legend(lu, labu)
    # ld = l3 + l4
    # labd = [l.get_label() for l in ld]
    # fig.ax_right_down.legend(ld, labd)

    fig.fig.tight_layout()

    fig.fig.savefig(save_path, bbox_inches='tight')
    return fig

def plot_control_comparison(database, entry_list, fig, color, save_path=None):

    for index, entry in enumerate(entry_list):
        #plot map
        bagplot.plotMap(database,fig.ax_left_up)
        bagplot.plotMapInflated(database,fig.ax_left_up)
        #plot path
        bagplot.plotPath(entry,fig.ax_left_up,color[index][0])
        #plot distance
        l1 = bagplot.plotDistance(entry,fig.ax_left_down,color[index][0],'position distance')

    fig.fig.tight_layout()

    fig.fig.savefig(save_path, bbox_inches='tight')
    return fig

def delete_entry(id):
    if database.deleteByFileId(id):
        logger.info("delete {0} successfully".format(id))

def plot_comparison_group(save_path=None):
    fig = FigureLayout("multiple_in_one_row")
    info_list = [('goal_position_error', 'Arrival Position Error(m)','Arrival Position Accuracy'),
                   ('goal_orientation_error','Arrival Orientation Error(rad)','Arrival Orientation Accuracy'),
                   ('time_to_goal', 'Navigation Time(s)', 'Navigation Time'),
                   ('detected_cmd_number', 'Detected Command Count', 'User Input Times'),
                   ('total_travel_distance', 'Total Drive Distance(m)', 'Navigation Distance'),
                   ]
                   #('workload', 'Experiment task workload'),]
    xlabels = ['(a)','(b)','(c)','(d)','(e)']
    for i, info in enumerate(info_list):
        plot_cross_comparison_bar(fig.ax_list[i], info[0], xlabels[i], info[1], info[2])

    fig.ax_list[0].legend(loc=(-0.9,0.4),fontsize=6)
    fig.fig.tight_layout()

    fig.fig.savefig(save_path, bbox_inches='tight')

def plot_steer_novelti():
    entry_list = [database.queryByFileId("sd-rob-novel-emotiv-c2a-1-2019_01_24_20_20_05"), 
                  database.queryByFileId("rui-rob-steer-emotiv-c2a-1-2019_01_23_19_52_59")]
    fig = FigureLayout('two_in_one_column')
    plot_control_comparison(database, entry_list, fig, [RED,BLUE], "/home/yaphes/Desktop/iros2019/c2a_novel_steering_comparison.pdf" )

def plot_novelti_entries():

    #a2b
    # entry_list = ['maozhen-rob-novel-emotiv-a2b-1-2019_01_28_20_28_20',
    #               'maozhong-rob-novel-emotiv-a2b-1-2019_02_22_15_36_36']
    
    # entry_list = [database.queryByFileId(entry) for entry in entry_list]
    # fig = FigureLayout('left_one_right_two_a2b')
    # plot_path_distance_entropy(database, entry_list,fig, [RED,BLUE],  "/home/yaphes/Desktop/iros2019/a2b_path_distance_entropy.pdf")

    #b2c
    # entry_list = ['jiahao-rob-novel-emotiv-b2c-1-2019_02_15_17_25_19', 
    #               'maozhong-rob-novel-emotiv-b2c-1-2019_02_22_15_40_12']
    # entry_list = [database.queryByFileId(entry) for entry in entry_list]
    # fig = FigureLayout('left_one_right_two_b2c')
    # plot_path_distance_entropy(database, entry_list,fig, [RED,BLUE], "/home/yaphes/Desktop/iros2019/b2c_path_distance_entropy.pdf")

    # #c2a
    entry_list = ['jiahao-rob-novel-emotiv-b2c-1-2019_02_15_17_25_19', 
                  'maozhong-rob-novel-emotiv-b2c-1-2019_02_22_15_40_12']
    entry_list = [database.queryByFileId(entry) for entry in entry_list]
    fig = FigureLayout('left_one_right_two_c2a')
    plot_path_distance_entropy(database, entry_list,fig, [RED,BLUE], "/home/yaphes/Desktop/iros2019/c2a_path_distance_entropy.pdf")

if __name__ == '__main__':
    if os.path.isfile(cache_path):
        with open(cache_path, 'rb') as cache:
            logger.info("Reading data from cache file %s" % cache_path)
            database = pickle.load(cache)
            #database.updateData()
    else:
        logger.info("No cache file found, start building a new one")
        #database = DataBase(data_dir, cache_path, cwavetool_path)
    
    # plot_comparison_group("/home/yaphes/storage/humane_paper/img/cross_comparison.pdf")

    # ca = CrossAnalysis(database)
    # workload_means, workload_std = ca.collect_workload_value(['novel-emotiv','novel-button','steer-emotiv'])
    # logger.info("Retrieved workload value")
    # logger.info(workload_means)
    # logger.info(workload_std)

    plot_novelti_entries()
    # plot_steer_novelti()
    # delete_entry('unver-rob-steer-emotiv-b2c-1-2019_02_27_18_57_37')
    plt.show()