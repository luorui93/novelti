#!/usr/bin/env python
# a bar plot with errorbars
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import csv
import os
import sys

#to fix: lthmi-auto-nav-experiment-2016-10-31_18-49-40.bag.txt



import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def drawTableWithBars(ax, means, stds, group_names, color_names):
    #matplotlib.rcParams.update({'font.size': 16})
    bar_colors = [  'yellow', '#ABABAB', '#5F9ED1', '#FF800E', '#006B40', 
                '#FFBC79', '#CFCFCF', '#C85200', '#A2C8EC', '#898989']
    gap = 0.5 # measured in bar width's
    n_colors = len(color_names)
    n_groups = len(group_names)
    width = 1.0 /(n_colors+gap)
    
    
    color_offsets = width*np.arange(n_colors)
    colors = [bar_colors[cid] for cid in xrange(n_colors)]
    
    group_barplots = []
    for gid, group_name in enumerate(group_names):
        barplot = ax.bar(gid+color_offsets, means[gid], width, color=colors, yerr=stds[gid])
        group_barplots.append(barplot)

    ax.set_xticks(np.arange((1.0-gap*width)/2, n_groups,1))
    ax.set_xticklabels(tuple(group_names))
    
    ax.legend(group_barplots[0], color_names,
        ncol=len(color_names),
        bbox_to_anchor=(0.0, 1.03, 1.0, 0.1), 
        loc=3,
        mode="expand", 
        borderaxespad=0.
    )
 

    ax.yaxis.grid(True)
    #ax.xaxis.grid(True)

def display4DdataAsBarPlotPage(title, means, stds, page_row_names, page_col_names, plot_group_names, plot_color_names):
    n_rows = len(page_row_names)
    n_cols = len(page_col_names)
    f, axarr = plt.subplots(n_rows, n_cols, sharex=True, facecolor='white', figsize=(16, 12))
    
    for page_row in range(n_rows):
        for page_col in range(n_cols):
            means2d = means[page_row, page_col, :, :]
            stds2d  = stds[page_row, page_col, :, :]
            if n_rows==1:
                if n_cols==1:
                    ax = axarr
                else:
                    ax = axarr[page_col]
            elif n_cols==1:
                ax = axarr[page_row]
            else:
                ax = axarr[page_row, page_col]
            drawTableWithBars(ax, means2d, stds2d, plot_group_names, plot_color_names)
            if page_row==0:
                ax.set_title(page_col_names[page_col], y=1.15)
            if page_col==0:
                ax.set_ylabel(page_row_names[page_row])
    f.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout(pad=1.0, h_pad=4.0, w_pad=2.0, rect=(0, 0, 1, 0.95))
    plt.show()



class DataTable:
    """
    Represents the data in the follwoing  format

    col1    col2    col3    col4
    qwe     0.123   prm1    12
    sad     12.23   prm2    43
    ....

    Data are saved as a dict of lists:
    self.data = {
        'col1': ['qwe', 'sad'],
        'col2': [0.123, 12.23],
        'col3': ['prm1', 'prm2'],
        'col4': [12, 43]
    }
    """
    
    def __init__(self, filter_dict={}):
        self.nrows = 0
        self.data = {}
        self.filt = filter_dict
    
    def isValidValue(self, key, val):
        if key in self.filt:
            if callable(self.filt[key]):
                return self.filt[key](val)
            else:
                return val==self.filt[key]
        return True
    
    def addDataRow(self, keys, data_row):
        for k, val in enumerate(data_row):
            if not self.isValidValue(keys[k], val):
                return False
        for k, val in enumerate(data_row):
            self.data[keys[k]].append(val)
        return True
    
    def addRow(self, keys, row):
        """
        returns True if row added
        """
        for k,val in enumerate(row):
            try:                #try to parse as int
                row[k] = int(val)
            except ValueError: #if fails, try to pass as float
                try:
                    row[k] = float(val)
                except ValueError: #if fails, keep as string
                    pass
        return self.addDataRow(keys,row)
    
    def addFromCsvFile(self, fname):
        """
        add data from a CSV-file named fname
        """
        with open(fname) as f:
            # reader header
            header = f.next()
            cur_keys = header.split()
            for key in cur_keys:
                if key not in self.data:
                    self.data[key] = [None]*self.nrows
            self.missingKeys = set(self.data.keys()) - set(cur_keys)
            
            #read data
            for line in f:
                if self.addRow(cur_keys, line.split()):
                    self.nrows += 1
                    for key in self.missingKeys:
                        self.data[key].append(None)

    @classmethod
    def fromCsvFiles(cls, fnames, filter_dict={}):
        """
        list of CSV-file names
        """
        self = cls(filter_dict)
        for fname in fnames:
            self.addFromCsvFile(fname)
        return self
    
    
class Table2NDimVector:
    
    def __init__(self, dtable, dim_names, result_cols):
        """
        Constucts a FiveDData object that represents the table data as an N-dimensional vector
        of mean values and standard deviations (corrected sample standard deviations).
        N-1 dimensions conrrespond to parameters and 1 dimension corresponds to the specific result value.
        paramconsider a data table (dtable):
        :param dtable: Data table of the following format
        prm_1    prm_2    prm_3     ... prm_NPRM    res_1   res_2   ... res_NRES
        "qwe"    "ddd"    "aaaa"        4           0.1     0.23        123.12
        "asd"    "eee"    "bbbb"        7           1.1     6.12        2345.98
        Here prm_K are experement setup parameters, and res_K are value measured in the experiment
        :param dim_names: List of N strings that represent names of the dimensions (in the desired order): 
            N-1 elements in the list should be parameter names, and 
            1 element name shoud be "_res_" which represents the result values
            example:
                ["_res_", "prm_3", "prm_1", "prm_7"]
        :param result_cols: List of column names from the table that represent results (measured values)
            that we want to include in the N-dim vector.
        :returns: nothing
        """
        self.readDimensions(dtable, dim_names, result_cols)
        #print(self.dim_sizes)
        self.dim_names  = dim_names
        self.calcMeansAndStds(dtable, dim_names, result_cols)

    def readDimensions(self, dtable, dim_names, result_cols):
        self.res_dim_k = next(k for k,name in enumerate(dim_names) if name=="_res_")
        self.idx2val = [[] for k,v in enumerate(dim_names)]
        self.idx2val[self.res_dim_k] = result_cols
        #read possible values
        for row_k in range(dtable.nrows):
            for dim_k, dim_name in enumerate(dim_names):
                if dim_k!=self.res_dim_k:
                    val = dtable.data[dim_name][row_k]
                    if not (val in self.idx2val[dim_k]):
                        self.idx2val[dim_k].append(val)
        #sort parameter values and create val2idx mapping:
        self.val2idx = [{} for k,v in enumerate(dim_names)]
        self.val2idx[self.res_dim_k] = {v:k for k,v in enumerate(result_cols)}
        for dim_k, dim_name in enumerate(dim_names):
            if dim_k != self.res_dim_k:
                self.idx2val[dim_k].sort()
                self.val2idx[dim_k] = {v:k for k,v in enumerate(self.idx2val[dim_k])}
        self.dim_sizes = tuple([len(v) for v in self.idx2val])
    
    def multiindexToStr(ix):
        return 
    
    def calcMeansAndStds(self, dtable, dim_names, result_cols):
        """using online variance calculation (verified by myself):
        https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
        Best explanation of the source of bias when calculating sample variance value
        https://en.wikipedia.org/wiki/Bessel%27s_correction#Source_of_bias"""
        self.nums  = np.zeros(self.dim_sizes, dtype=np.float) #5D array
        self.means = np.zeros(self.dim_sizes, dtype=np.float) #5D array
        self.m2s   = np.zeros(self.dim_sizes, dtype=np.float) #5D array
        self.stds  = np.zeros(self.dim_sizes, dtype=np.float) # corrected sample standard deviation https://en.wikipedia.org/wiki/Standard_deviation#Corrected_sample_standard_deviation 
        for row_k in range(dtable.nrows):
            index = [0]*len(dim_names)
            for dim_k, dim_name in enumerate(dim_names):
                if dim_k != self.res_dim_k:
                    val = dtable.data[dim_name][row_k]
                    index[dim_k] = self.val2idx[dim_k][val]
            for res_k, res_name in enumerate(result_cols):
                index[self.res_dim_k] = res_k
                ix = tuple(index)
                #print ix
                self.nums[ix] += 1
                x = dtable.data[res_name][row_k]
                print "%s=%s" % (res_name, str(x))
                delta = x - self.means[ix]
                self.means[ix] += delta / self.nums[ix]
                self.m2s[ix] = delta * (x - self.means[ix])
        it = np.nditer(self.m2s, flags=['multi_index'])
        while not it.finished:
            if self.nums[it.multi_index]>=2:
                self.stds[it.multi_index] = np.sqrt(self.m2s[it.multi_index] / (self.nums[it.multi_index]-1))
            it.iternext()
        print self.means

    def display4dPage(self, firstDimValue, title=None):
        varIdx = self.val2idx[0][firstDimValue]
        if title is None:
            title = "Various characteristics, %s=%s" %(self.dim_names[0], str(self.idx2val[0][varIdx]))
        display4DdataAsBarPlotPage(title, self.means[varIdx], self.stds[varIdx], \
            self.idx2val[1], self.idx2val[2], self.idx2val[3], self.idx2val[4])
                      

    
