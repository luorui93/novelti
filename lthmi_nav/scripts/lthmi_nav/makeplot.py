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
    dt = DataTable.fromCsvFiles(files, filter_dict={'pos': lambda method: method in ["no_move", "ra_maxprob", "maxprob_obst", "nearcog_obst", "cog2lopt", "ramaxprob2lopt", "maxprob2lopt"]})
    
    dims = ('mx', '_res_', 'pos','div','path') # 5 dimensions
    vals = ('t_pdf', 't_pos', 't_div')#, 'over_len', 'sep2nav', 'over_time')
    #vals = ('sep2nav',)
    v = Table2NDimVector(dt, dims, vals)
    v.display4dPage('mx70')

