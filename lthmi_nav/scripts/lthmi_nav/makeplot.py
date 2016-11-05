#!/usr/bin/env python
import os
import sys


from TableDataVisualizer import *

usage = """ USAGE:
    ./makeplot.py stats_file_path1.txt [ stats_file_path2.txt [stats_file_path3.txt ... ]]

"""

if __name__=="__main__":
    if len(sys.argv)<2:
        print usage
        sys.exit(1)
        
    files = sys.argv[1:]
    #print files
    dt = DataTable.fromCsvFiles(files)#, filter_dict={'pos':'maxprob_obst', 'div' : 'htile', 'path':110})
    
    dims = ('mx', '_res_', 'pos','div','path') # 5 dimensions
    vals = ('over_len', 'sep2nav', 'over_time')
    #vals = ('sep2nav',)
    v = Table2NDimVector(dt, dims, vals)
    #print dd.stds
    title = "Various characterestics, %s=%s" %(v.dim_names[0], str(v.idx2val[0][0]))
    display4DdataAsBarPlotPage(title, v.means[0], v.stds[0], v.idx2val[1], v.idx2val[2], v.idx2val[3], v.idx2val[4])
