#!/usr/bin/env python
import os
import sys


from TableDataVisualizer import *

usage = """ USAGE:
    ./makeplot.py <output_image_files_prefix> stats_file_path1.txt [ stats_file_path2.txt [stats_file_path3.txt ... ]]

"""

"""
./makeplot.py ~/Desktop/lthmi_nav_data/sim/amazon/stats/1/*.txt ~/Desktop/lthmi_nav_data/sim/amazon/stats/2/*.txt ~/Desktop/lthmi_nav_data/sim/amazon/stats/3/*.txt ~/Desktop/lthmi_nav_data/sim/amazon/stats/4/*.txt ~/Desktop/lthmi_nav_data/sim/amazon/stats/5/*.txt ~/Desktop/lthmi_nav_data/sim/amazon/stats/6/*.txt ~/Desktop/lthmi_nav_data/sim/amazon/stats/7/*.txt ~/Desktop/lthmi_nav_data/sim/amazon/stats/8/*.txt 
"""


def fff(self, d):
    print d['t_div']
    print d['t_nav']
    print d['pos']
    print d['div']
    print self.cur_file
    print d['t_div']/d['t_nav']
    return d['t_nav'] #1 if d['t_nav']==0.0 else 0

if __name__=="__main__":
    if len(sys.argv)<3:
        print usage
        sys.exit(1)
        

    output_image_files_prefix =  sys.argv[1]
    files = sys.argv[2:]
    #print files
    dt = DataTable.fromCsvFiles(files, 
        filter_dict = {
            #'pos': lambda method: method in ["no_move", "ra_maxprob", "maxprob_obst", "nearcog_obst", "cog2lopt", "ramaxprob2lopt", "maxprob2lopt", "gopt"]
            'pos': lambda method: method in ["no_move", "ra_maxprob", "maxprob_obst", "nearcog_obst", "cog2lopt"]
            
        },
        calculated_values = {
            #'t_div_': lambda self, d: d['t_div']/d['t_nav'],
            #'t_pos_': lambda self, d: d['t_pos']/d['t_nav'],
            #'t_pdf_': lambda self, d: d['t_pdf']/d['t_nav'],
            't_nav_corr': lambda self, d: d['t_nav']-d['t_pos']-d['t_div'],
        }
    )
    print "Total number of lines in the data table = %d"  % dt.getNumberOfRecords()
    print "All matrices: %s" % (str(dt.getUniqueRecordsInColumn("mx")))
    print "All velocities: %s" % (str(dt.getUniqueRecordsInColumn("vel")))
    print "All periods: %s" % (str(dt.getUniqueRecordsInColumn("period")))
    print "All divs: %s" % (str(dt.getUniqueRecordsInColumn("div")))
    v = Table2NDimVector(dt, 
        dim_names   = ('mx', '_res_', 'pos','path','div'),  # 5 dimensions 
        result_cols = ('t_nav', 't_nav_corr') # ,'over_len', 'vel', 'period', 'sep2nav', 'over_time'
    )
    renames = {
            "plot_group_names":  {
                100: "route 1", 
                103: "route 2", 
                110: "route 3"
            },
            "page_row_names": {
                'over_len'  : 'driven distance/shortest distance', 
                't_nav'     :'navigation duration, sec', 
                't_nav_corr':'navigation duration, sec\n (calculation time excluded)'
            },
            #"page_col_names": {
                #"no_move"       : "no move until inferred", 
                #"ra_maxprob"    : "max prob in reach area", 
                #"maxprob_obst"  : "closest to max prob", 
                #"nearcog_obst"  : "closest to COG", 
                #"cog2lopt"      : "local optimum"
            #},
            #"plot_color_names": {
                #'htile'     : "horizontal tile",
                #'vtile'     : "vertical tile", 
                #'altertile' : "alternating tile",
                #'extremal'  : "by extremals",
                #'equidist'  : "by equdistants",
                #"extredist" : "extremals and equidistants",
            #}
    }
    
    w,h = 16, 7.5
    (fig, plt1, axarr) = v.render4dPage('mx70', "HMI matrix: 70%", renames)
    fig.set_size_inches(w, h)
    plt1.savefig(output_image_files_prefix+"mx70.pdf")


    (fig, plt1, axarr) = v.render4dPage('mx79', "HMI matrix: 79%", renames)
    fig.set_size_inches(w, h)
    plt1.savefig(output_image_files_prefix+"mx79.pdf")

    (fig, plt1, axarr) = v.render4dPage('mx91', "HMI matrix: 91%", renames)
    fig.set_size_inches(w, h)
    plt1.savefig(output_image_files_prefix+"mx91.pdf")
    
    (fig, plt1, axarr) = v.render4dPage('mx100', "HMI matrix: 100% (deterministic HMI)", renames)
    fig.set_size_inches(w, h)
    plt1.savefig(output_image_files_prefix+"mx100.pdf")
