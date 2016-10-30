#!/usr/bin/env python
# a bar plot with errorbars
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import csv
import os
import sys


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
            try:                #try to parse as a number
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
    def fromCsvFiles(cls, fnames):
        """
        list of CSV-file names
        """
        self = cls()
        for fname in fnames:
            self.addFromCsvFile(fname)
        return self
    
class FiveDData:
    
    def __init__(self, dtable, params, results):
        """
        len(params) must be =4
        len(results) is equalt to the number of columns to analyze as rersult values
        From the 2D table, generates a 5D array: data[param1][param2][param3][param4][result]
        """
        self.readDimensions(dtable, params, results)
        self.sizes = (len(self.dims[params[0]]),
                      len(self.dims[params[1]]),
                      len(self.dims[params[2]]),
                      len(self.dims[params[3]]),
                      len(results))
        print(self.sizes)
        self.params  = params
        self.results = results
        self.calcMeansAndStds(dtable, params, results)

    def readDimensions(self, dtable, params, results):
        self.dims = {key:{} for key in params}
        for rowid in range(dtable.nrows):
            for prmid, prm in enumerate(params):
                val = dtable.data[prm][rowid]
                if not (val in self.dims[prm]):
                    self.dims[prm][val] = len(self.dims[prm])
        self.dims['result'] = {v: k for (k, v) in enumerate(results)}
    
    def names2indexes(self, n0, n1, n2, n3, res):
        return (self.dims[self.params[0]][n0],
                self.dims[self.params[1]][n1],
                self.dims[self.params[2]][n2],
                self.dims[self.params[3]][n3],
                res)
        
    def calcMeansAndStds(self, dtable, params, results):
        """using online variance calculation (verified by myself):
        https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
        Best explanation of the source of bias when calculating sample variance value
        https://en.wikipedia.org/wiki/Bessel%27s_correction#Source_of_bias"""
        self.nums  = np.zeros(self.sizes, dtype=np.float) #5D array
        self.means = np.zeros(self.sizes, dtype=np.float) #5D array
        self.m2s   = np.zeros(self.sizes, dtype=np.float) #5D array
        self.stds  = np.zeros(self.sizes, dtype=np.float) # corrected sample standard deviation https://en.wikipedia.org/wiki/Standard_deviation#Corrected_sample_standard_deviation 
        for r in range(dtable.nrows):
            for resid, res in enumerate(results):
                ix = self.names2indexes(
                    dtable.data[params[0]][r],
                    dtable.data[params[1]][r],
                    dtable.data[params[2]][r],
                    dtable.data[params[3]][r],
                    resid)
                #print ix
                self.nums[ix] += 1
                x = dtable.data[res][r]
                delta = x - self.means[ix]
                self.means[ix] += delta / self.nums[ix]
                self.m2s[ix] = delta * (x - self.means[ix])
        it = np.nditer(self.m2s, flags=['multi_index'])
        while not it.finished:
            if self.nums[it.multi_index]>=2:
                self.stds[it.multi_index] = self.m2s[it.multi_index] / (self.nums[it.multi_index]-1)
            it.iternext()


    """def get(v0,v1,v1,v3,v4):
        k0 = self.dims[params[0]][v0]
        k1 = self.dims[params[1]][v1]
        k2 = self.dims[params[2]][v2]
        k3 = self.dims[params[3]][v3]
        k4 = self.dims[params[4]][v4]
        return self.sd[k0][k1][k2][k3][k4]

    def upd(v0,v1,v1,v3,v4, new_value):
        k0 = self.dims[params[0]][v0]
        k1 = self.dims[params[1]][v1]
        k2 = self.dims[params[2]][v2]
        k3 = self.dims[params[3]][v3]
        k4 = self.dims[params[4]][v4]
        self.sd[k0][k1][k2][k3][k4] = new_value"""

"""
fname = "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-03-30.bag.txt"

data = np.genfromtxt(fname, #https://docs.scipy.org/doc/numpy/reference/generated/numpy.genfromtxt.html
    dtype=None, # None means determine individually
    comments='#',
    delimiter=None, #None means any whitespaces
    converters=None, #use default converters
    names=True, 
    case_sensitive=True, 
    loose=False,  #If True, do not raise errors for invalid values.
    invalid_raise=True) #If True, an exception is raised if an inconsistency is detected in the number of columns. If False, a warning is emitted and the offending lines are skipped.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def drawTableWithBars(means, stds, group_names, color_names):
    matplotlib.rcParams.update({'font.size': 16})
    bar_colors = [  'yellow', '#ABABAB', '#5F9ED1', '#FF800E', '#006B40', 
                '#FFBC79', '#CFCFCF', '#C85200', '#A2C8EC', '#898989']
    gap = 0.5 # measured in bar width's
    n_colors = len(color_names)
    n_groups = len(group_names)
    width = 1.0 /(n_colors+gap)
    
    
    color_offsets = width*np.arange(n_colors)
    colors = [bar_colors[cid] for cid in xrange(n_colors)]

    plt.figure(num=None, dpi=80, facecolor='w')  
    ax = plt.gca()
    
    group_barplots = []
    for gid, group_name in enumerate(group_names):
        barplot = ax.bar(gid+color_offsets, means[gid], width, color=colors, yerr=stds[gid])
        group_barplots.append(barplot)

    ax.set_xticks(np.arange((1.0-gap*width)/2, n_groups,1))
    ax.set_xticklabels(tuple(group_names))
    
    ax.legend(group_barplots[0], color_names,
        ncol=len(color_names),
        bbox_to_anchor=(0., 1.01, 1., .101), 
        loc=3,
        mode="expand", 
        borderaxespad=0.
    )
    #legend_patches = [mpatches.Patch(color=bar_colors[cid], label=color_names[cid]) for cid in xrange(n_colors)]
    #print legend_patches
    #group_barplots[0].legend(
        #handles=legend_patches#,
        #ncol=len(color_names), 
        #loc="upper center", 
        #bbox_to_anchor=(0.5, 1.135)
    #)
 

    ax.yaxis.grid(True)
    #ax.xaxis.grid(True)
    plt.show()
    return plt, ax



if __name__=="__main__":
    dims = ('pos','div','mx','path')
    vals = ('over_len', 'sep2nav', 'over_time')
    files = [
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-39-05.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-40-20.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-01-04.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-41-32.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-02-30.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-41-54.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-03-00.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-42-15.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-03-30.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-48-37.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-12-42.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-55-08.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-21-53.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-56-16.bag.txt",
        "/home/sd/Desktop/lthmi_nav_data/stats/lthmi-auto-nav-experiment-2016-10-28-17-23-21.bag.txt"]
    dt = DataTable.fromCsvFiles(files)
    
    dd = FiveDData(dt, dims, vals)
    #print dd.stds
    
    means = np.random.rand(2,5)
    stds = np.random.rand(2,5)/4.0
    
    drawTableWithBars(means, stds, ["gid1", "gid2"], ["cid1", "cid2", "cid3", "cid4", "cid5"])

    #t = np.arange(0.0, 2.0, 0.01)
    #s1 = np.sin(2*np.pi*t)
    #s2 = np.sin(4*np.pi*t)

    #plt.figure(1)
    #plt.subplot(211)
    #plt.plot(t, s1)
    #plt.subplot(212)
    #plt.plot(t, 2*s1)

    #plt.figure(2)
    #plt.plot(t, s2)

    ## now switch back to figure 1 and make some changes
    #plt.figure(1)
    #plt.subplot(211)
    #plt.plot(t, s2, 'gs')
    #ax = plt.gca()
    #ax.set_xticklabels([])

    #plt.show()
