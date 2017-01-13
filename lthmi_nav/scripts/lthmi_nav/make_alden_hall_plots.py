#!/usr/bin/env python
import os
import sys


from Bag2Plot import *

usage = """ USAGE:
    TODO

"""

if __name__=="__main__":
    baseDir = "/home/sd/Desktop/ah_data/2016-12-experiments-in-alden-hall/bags"
    
    bags = {
        "storage1to2" : {
            #div=nearcog_extremal, pos=no_move
            "2017-01-02_02-06-57_EST-0500" : "#000055",
            "2017-01-02_02-24-31_EST-0500" : "#000055",
            
            #div=nearcog_extremal, pos=cog2lopt
            "2017-01-02_08-20-50_EST-0500" : "#0000FF",
            "2017-01-02_08-32-56_EST-0500" : "#0000FF",
            
            #div=nearcog_extremal, pos=nearcog_obst
            "2017-01-02_08-50-45_EST-0500" : "#CCCCFF",
            "2017-01-02_09-07-48_EST-0500" : "#CCCCFF",
            
            
            #div=altertile, pos=no_move
            "2017-01-02_09-20-26_EST-0500" : "#550000",
            "2017-01-02_09-32-59_EST-0500" : "#550000",
            
            #div=altertile, pos=cog2lopt
            "2017-01-02_09-45-48_EST-0500" : "#FF0000",
            "2017-01-02_09-59-49_EST-0500" : "#FF0000",
            
            #div=altertile, pos=nearcog_obst
            "2017-01-02_10-29-59_EST-0500" : "#FFCCCC",
            "2017-01-02_10-40-40_EST-0500" : "#FFCCCC",
            
            
            #div=extredist, pos=no_move
            "2017-01-02_10-51-22_EST-0500" : "#005500",
            "2017-01-02_11-08-09_EST-0500" : "#005500",
            
            #div=extredist, pos=cog2lopt
            "2017-01-02_11-40-14_EST-0500" : "#00FF00",
            "2017-01-02_11-59-34_EST-0500" : "#00FF00",
            
            #div=extredist, pos=nearcog_obst
            "2017-01-02_12-11-18_EST-0500" : "#CCFFCC",
            "2017-01-02_12-31-47_EST-0500" : "#CCFFCC",
        },
        "storage2to3" : {
            #div=nearcog_extremal, pos=no_move
            "2017-01-02_02-08-49_EST-0500" : "#000055",
            "2017-01-02_02-25-56_EST-0500" : "#000055",
            
            #div=nearcog_extremal, pos=cog2lopt
            "2017-01-02_08-22-44_EST-0500" : "#0000FF",
            "2017-01-02_08-35-00_EST-0500" : "#0000FF",
            
            #div=nearcog_extremal, pos=nearcog_obst
            "2017-01-02_08-58-57_EST-0500" : "#CCCCFF",
            "2017-01-02_09-09-07_EST-0500" : "#CCCCFF",
            
            #div=altertile, pos=no_move
            "2017-01-02_09-22-03_EST-0500" : "#550000",
            "2017-01-02_09-35-51_EST-0500" : "#550000",
            
            #div=altertile, pos=cog2lopt
            "2017-01-02_09-47-25_EST-0500" : "#FF0000",
            "2017-01-02_10-01-29_EST-0500" : "#FF0000",
            
            #div=altertile, pos=nearcog_obst
            "2017-01-02_10-31-45_EST-0500" : "#FFCCCC",
            "2017-01-02_10-42-08_EST-0500" : "#FFCCCC",
            
            
            #div=extredist, pos=no_move
            "2017-01-02_10-52-55_EST-0500" : "#005500",
            "2017-01-02_11-09-39_EST-0500" : "#005500",
            
            #div=extredist, pos=cog2lopt
            "2017-01-02_11-41-51_EST-0500" : "#00FF00",
            "2017-01-02_12-02-03_EST-0500" : "#00FF00",
            
            #div=extredist, pos=nearcog_obst
            "2017-01-02_12-12-49_EST-0500" : "#CCFFCC",
            "2017-01-02_12-32-56_EST-0500" : "#CCFFCC",
        },
        "storage3tolivroom2" : {
            ##div=nearcog_extremal, pos=no_move
            #"2017-01-02_02-14-36_EST-0500" : "#000055",
            #"2017-01-02_08-06-52_EST-0500" : "#000055",
            
            #div=nearcog_extremal, pos=cog2lopt
            "2017-01-02_08-25-42_EST-0500" : "#0000FF",
            "2017-01-02_08-36-23_EST-0500" : "#0000FF",
            
            #div=nearcog_extremal, pos=nearcog_obst
            "2017-01-02_09-00-40_EST-0500" : "#CCCCFF",
            "2017-01-02_09-10-30_EST-0500" : "#CCCCFF",
            
            ##div=altertile, pos=no_move
            #"2017-01-02_09-23-47_EST-0500" : "#550000",
            #"2017-01-02_09-37-15_EST-0500" : "#550000",
            
            #div=altertile, pos=cog2lopt
            "2017-01-02_09-52-07_EST-0500" : "#FF0000",
            "2017-01-02_10-03-51_EST-0500" : "#FF0000",
            
            #div=altertile, pos=nearcog_obst
            "2017-01-02_10-33-59_EST-0500" : "#FFCCCC",
            "2017-01-02_10-43-49_EST-0500" : "#FFCCCC",
            
            
            ##div=extredist, pos=no_move
            #"2017-01-02_10-54-29_EST-0500" : "#005500",
            #"2017-01-02_11-11-00_EST-0500" : "#005500",
            
            #div=extredist, pos=cog2lopt
            "2017-01-02_11-52-24_EST-0500" : "#00FF00",
            "2017-01-02_12-03-34_EST-0500" : "#00FF00",
            
            #div=extredist, pos=nearcog_obst
            "2017-01-02_12-14-14_EST-0500" : "#CCFFCC",
            "2017-01-02_12-34-12_EST-0500" : "#CCFFCC",
        },
        "livroom2to1" : {
            #div=nearcog_extremal, pos=no_move
            "2017-01-02_02-19-02_EST-0500" : "#000055",
            "2017-01-02_08-10-09_EST-0500" : "#000055",
            
            #div=nearcog_extremal, pos=cog2lopt
            "2017-01-02_08-28-40_EST-0500" : "#0000FF",
            "2017-01-02_08-40-09_EST-0500" : "#0000FF",
            
            #div=nearcog_extremal, pos=nearcog_obst
            "2017-01-02_09-04-00_EST-0500" : "#CCCCFF",
            "2017-01-02_09-12-43_EST-0500" : "#CCCCFF",
            
            #div=altertile, pos=no_move
            "2017-01-02_09-27-10_EST-0500" : "#550000",
            "2017-01-02_09-40-13_EST-0500" : "#550000",
            
            #div=altertile, pos=cog2lopt
            "2017-01-02_09-54-57_EST-0500" : "#FF0000",
            "2017-01-02_10-05-41_EST-0500" : "#FF0000",
            
            #div=altertile, pos=nearcog_obst
            "2017-01-02_10-35-50_EST-0500" : "#FFCCCC",
            "2017-01-02_10-45-33_EST-0500" : "#FFCCCC",
            
            
            #div=extredist, pos=no_move
            "2017-01-02_10-58-15_EST-0500" : "#005500",
            "2017-01-02_11-13-45_EST-0500" : "#005500",
            
            #div=extredist, pos=cog2lopt
            "2017-01-02_11-54-43_EST-0500" : "#00FF00",
            "2017-01-02_12-05-31_EST-0500" : "#00FF00",
            
            #div=extredist, pos=nearcog_obst
            "2017-01-02_12-16-08_EST-0500" : "#CCFFCC",
            "2017-01-02_12-36-20_EST-0500" : "#CCFFCC",
        },
        "livroom1tokitchen1" : {
            #div=nearcog_extremal, pos=no_move
            "2017-01-02_02-20-53_EST-0500" : "#000055",
            "2017-01-02_08-12-32_EST-0500" : "#000055",
            
            #div=nearcog_extremal, pos=cog2lopt
            "2017-01-02_08-29-55_EST-0500" : "#0000FF",
            "2017-01-02_08-41-33_EST-0500" : "#0000FF",
            
            #div=nearcog_extremal, pos=nearcog_obst
            "2017-01-02_09-05-15_EST-0500" : "#CCCCFF",
            "2017-01-02_09-15-12_EST-0500" : "#CCCCFF",
            
            #div=altertile, pos=no_move
            "2017-01-02_09-29-30_EST-0500" : "#550000",
            "2017-01-02_09-42-01_EST-0500" : "#550000",
            
            #div=altertile, pos=cog2lopt
            "2017-01-02_09-57-35_EST-0500" : "#FF0000",
            "2017-01-02_10-07-50_EST-0500" : "#FF0000",
            
            #div=altertile, pos=nearcog_obst
            "2017-01-02_10-37-17_EST-0500" : "#FFCCCC",
            "2017-01-02_10-47-38_EST-0500" : "#FFCCCC",
            
            
            #div=extredist, pos=no_move
            "2017-01-02_11-03-28_EST-0500" : "#005500",
            "2017-01-02_11-35-47_EST-0500" : "#005500",
            
            #div=extredist, pos=cog2lopt
            "2017-01-02_11-56-37_EST-0500" : "#00FF00",
            "2017-01-02_12-07-39_EST-0500" : "#00FF00",
            
            #div=extredist, pos=nearcog_obst
            "2017-01-02_12-17-58_EST-0500" : "#CCFFCC",
            "2017-01-02_12-38-16_EST-0500" : "#CCFFCC",
        }
    }
    
    t2f = TimeStrToFilepath(baseDir)
    
    p = LthmiNavExpPlot()
    
    for timeStr, color in bags["storage3tolivroom2"].iteritems():
        filepath = t2f.getPath(timeStr)
        bagRecord = LthmiNavExpRecord(filepath)
        p.plotBagRecord(bagRecord, color)
        
    p.show()
    