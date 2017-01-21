#!/usr/bin/env python
import os
import sys


from Bag2Plot import *

usage = """ USAGE:
    TODO

"""

bagDir = "/home/sd/Desktop/ah_data/2016-12-experiments-in-alden-hall/bags"
cacheDir = "/home/sd/Desktop/ah_data_cache"
cwaveToolPath = "/home/sd/ws/devel/lib/lthmi_nav/cwave_cmdline"

DBASE = RecordByStamp(bagDir, cacheDir, cwaveToolPath)

bags = {
    "mx94_no_pois": {
        "door1tolivroom1":      ['2016-12-28_18-37-51_EST-0500', '2016-12-28_19-22-15_EST-0500', '2016-12-28_19-37-11_EST-0500'],
        "livroom1tooffice1":    ['2016-12-28_19-04-00_EST-0500', '2016-12-28_19-25-33_EST-0500', '2016-12-28_19-39-14_EST-0500'],
        "office1tobathroom1":   ['2016-12-28_19-06-45_EST-0500', '2016-12-28_19-27-39_EST-0500', '2016-12-28_19-42-09_EST-0500'],
        "bathroom1tomusic1":    ['2016-12-28_19-08-54_EST-0500', '2016-12-28_19-29-43_EST-0500', '2016-12-28_19-44-07_EST-0500'],
        "music1tobedroom1":     ['2016-12-28_19-13-02_EST-0500', '2016-12-28_19-32-00_EST-0500', '2016-12-28_19-46-12_EST-0500'],
        "bedroom1todoor2":      ['2016-12-28_19-15-54_EST-0500', '2016-12-28_19-33-51_EST-0500', '2016-12-28_19-48-13_EST-0500'],
    },
    "mx70_no_pois": {
        "door1tolivroom1":      ['2016-12-29_13-57-10_EST-0500', '2016-12-29_14-24-06_EST-0500', '2016-12-29_14-43-48_EST-0500'],
        "livroom1tooffice1":    ['2016-12-29_14-07-56_EST-0500', '2016-12-29_14-26-31_EST-0500', '2016-12-29_14-47-11_EST-0500'],
        "office1tobathroom1":   ['2016-12-29_14-10-07_EST-0500', '2016-12-29_14-29-05_EST-0500', '2016-12-29_14-50-31_EST-0500'],
        "bathroom1tomusic1":    ['2016-12-29_14-12-55_EST-0500', '2016-12-29_14-31-31_EST-0500', '2016-12-29_14-53-28_EST-0500'],
        "music1tobedroom1":     ['2016-12-29_14-16-31_EST-0500', '2016-12-29_14-37-43_EST-0500', '2016-12-29_14-56-45_EST-0500'],
        "bedroom1todoor2":      ['2016-12-29_14-20-02_EST-0500', '2016-12-29_14-40-23_EST-0500', '2016-12-29_15-00-09_EST-0500'],
    },
    
    "mx94_pois_NOT_FULLY_CORRECT": {
        "door1tolivroom1":      ['2016-12-29_19-24-40_EST-0500', '2016-12-29_21-58-56_EST-0500', ],
        "livroom1tooffice1":    [                                '2016-12-29_22-01-52_EST-0500', ],
        "office1tobathroom1":   ['2016-12-29_21-47-11_EST-0500', '2016-12-29_22-04-07_EST-0500', ],
        "bathroom1tomusic1":    ['2016-12-29_21-50-39_EST-0500', '2016-12-29_22-06-31_EST-0500', ],
        "music1tobedroom1":     ['2016-12-29_21-53-46_EST-0500', '2016-12-29_22-09-43_EST-0500', ],
        "bedroom1todoor2":      ['2016-12-29_21-55-19_EST-0500', '2016-12-29_22-11-02_EST-0500', ],
    },
    "mx70_pois_NOT_FULLY_CORRECT": {
        "door1tolivroom1":      ['2016-12-29_22-15-07_EST-0500', '2016-12-29_22-41-22_EST-0500', ],
        "livroom1tooffice1":    ['2016-12-29_22-19-14_EST-0500', '2016-12-29_22-44-42_EST-0500', ],
        "office1tobathroom1":   ['2016-12-29_22-21-10_EST-0500', '2016-12-29_22-55-06_EST-0500', ],
        "bathroom1tomusic1":    ['2016-12-29_22-23-51_EST-0500', '2016-12-29_22-58-09_EST-0500', ],
        "music1tobedroom1":     ['2016-12-29_22-27-11_EST-0500', '2016-12-29_23-00-51_EST-0500', ],
        "bedroom1todoor2":      ['2016-12-29_22-29-59_EST-0500', '2016-12-29_23-02-59_EST-0500', ],
    },
    "mx70_pois": {
        "door1tolivroom1":      ['2016-12-31_11-25-58_EST-0500', '2016-12-31_11-41-49_EST-0500', '2016-12-31_11-55-26_EST-0500'],
        "livroom1tooffice1":    ['2016-12-31_11-29-06_EST-0500', '2016-12-31_11-44-23_EST-0500', '2016-12-31_11-57-43_EST-0500'],
        "office1tobathroom1":   ['2016-12-31_11-31-20_EST-0500', '2016-12-31_11-46-14_EST-0500', '2016-12-31_11-59-42_EST-0500'],
        "bathroom1tomusic1":    ['2016-12-31_11-33-41_EST-0500', '2016-12-31_11-48-38_EST-0500', '2016-12-31_12-02-04_EST-0500'],
        "music1tobedroom1":     ['2016-12-31_11-36-35_EST-0500', '2016-12-31_11-50-55_EST-0500', '2016-12-31_12-04-25_EST-0500'],
        "bedroom1todoor2":      ['2016-12-31_11-38-41_EST-0500', '2016-12-31_11-52-47_EST-0500', '2016-12-31_12-05-58_EST-0500'],
    },
    "mx94_pois": {
        "door1tolivroom1":      ['2016-12-31_12-10-00_EST-0500', '2016-12-31_12-23-13_EST-0500', '2016-12-31_12-52-15_EST-0500'],
        "livroom1tooffice1":    ['2016-12-31_12-12-48_EST-0500', '2016-12-31_12-25-14_EST-0500', '2016-12-31_12-55-05_EST-0500'],
        "office1tobathroom1":   ['2016-12-31_12-14-42_EST-0500', '2016-12-31_12-27-04_EST-0500', '2016-12-31_12-57-00_EST-0500'],
        "bathroom1tomusic1":    ['2016-12-31_12-16-31_EST-0500', '2016-12-31_12-28-46_EST-0500', '2016-12-31_12-58-58_EST-0500'],
        "music1tobedroom1":     ['2016-12-31_12-18-52_EST-0500', '2016-12-31_12-30-55_EST-0500', '2016-12-31_13-01-23_EST-0500'],
        "bedroom1todoor2":      ['2016-12-31_12-20-06_EST-0500', '2016-12-31_12-32-07_EST-0500', '2016-12-31_13-02-38_EST-0500'],
    },
    
    "change_of_mind_mx85_no_pois": {
        "kitchen1storage1":     ['2016-12-30_23-52-22_EST-0500', '2016-12-31_00-02-24_EST-0500', '2016-12-31_00-10-05_EST-0500', '2016-12-31_00-18-06_EST-0500', '2016-12-31_00-26-40_EST-0500'],
        "storage1tostorage3":   ['2016-12-30_23-56-01_EST-0500', '2016-12-31_00-06-22_EST-0500', '2016-12-31_00-13-25_EST-0500', '2016-12-31_00-21-30_EST-0500', '2016-12-31_00-30-31_EST-0500']
    },
    "change_of_mind_mx85_with_pois": {
        "kitchen1storage1":     ['2016-12-31_13-25-26_EST-0500', '2016-12-31_13-32-25_EST-0500', '2016-12-31_13-39-14_EST-0500', '2016-12-31_13-46-34_EST-0500', '2016-12-31_14-01-07_EST-0500'],
        "storage1tostorage3":   ['2016-12-31_13-28-36_EST-0500', '2016-12-31_13-36-11_EST-0500', '2016-12-31_13-42-05_EST-0500', '2016-12-31_13-52-59_EST-0500', '2016-12-31_14-04-18_EST-0500']
    },
    
    "nav_to_non_poi_mx94": {
        "kitchen1tooffice1":    ['2016-12-31_14-30-27_EST-0500', '2016-12-31_14-35-13_EST-0500', '2016-12-31_14-39-36_EST-0500', '2016-12-31_14-44-14_EST-0500'],
        "office1tobedroom1":    ['2016-12-31_14-32-59_EST-0500', '2016-12-31_14-37-20_EST-0500', '2016-12-31_14-41-48_EST-0500', '2016-12-31_14-46-18_EST-0500'],
    },
    "nav_to_non_poi_mx70": {
        "kitchen1tooffice1":    ['2016-12-31_18-18-12_EST-0500','2016-12-31_18-23-53_EST-0500' , '2016-12-31_18-39-26_EST-0500', '2016-12-31_18-51-16_EST-0500'],
        "office1tobedroom1":    ['2016-12-31_18-20-36_EST-0500', '2016-12-31_18-35-12_EST-0500', '2016-12-31_18-46-21_EST-0500', '2016-12-31_18-54-57_EST-0500'],
    },
    
    "two_mx_no_pois": {
        "door1tolivroom1":      ['2017-01-01_15-07-57_EST-0500', '2017-01-01_15-25-35_EST-0500', '2017-01-01_15-41-32_EST-0500'],
        "livroom1tooffice1":    ['2017-01-01_15-10-50_EST-0500', '2017-01-01_15-27-34_EST-0500', '2017-01-01_15-43-32_EST-0500'],
        "office1tobathroom1":   ['2017-01-01_15-12-38_EST-0500', '2017-01-01_15-29-20_EST-0500', '2017-01-01_15-45-12_EST-0500'],
        "bathroom1tomusic1":    ['2017-01-01_15-15-14_EST-0500', '2017-01-01_15-31-24_EST-0500', '2017-01-01_15-47-21_EST-0500'],
        "music1tobedroom1":     ['2017-01-01_15-18-52_EST-0500', '2017-01-01_15-33-22_EST-0500', '2017-01-01_15-49-24_EST-0500'],
        "bedroom1todoor2":      ['2017-01-01_15-22-01_EST-0500', '2017-01-01_15-35-14_EST-0500', '2017-01-01_15-51-19_EST-0500'],
    },
    "two_mx_with_pois": {
        "door1tolivroom1":      ['2017-01-01_16-04-07_EST-0500', '2017-01-01_16-16-37_EST-0500', '2017-01-01_17-21-22_EST-0500'],
        "livroom1tooffice1":    ['2017-01-01_16-06-35_EST-0500', '2017-01-01_16-18-33_EST-0500', '2017-01-01_17-23-25_EST-0500'],
        "office1tobathroom1":   ['2017-01-01_16-08-14_EST-0500', '2017-01-01_17-13-57_EST-0500', '2017-01-01_17-25-08_EST-0500'],
        "bathroom1tomusic1":    ['2017-01-01_16-09-47_EST-0500', '2017-01-01_17-15-59_EST-0500', '2017-01-01_17-26-49_EST-0500'],
        "music1tobedroom1":     ['2017-01-01_16-11-46_EST-0500', '2017-01-01_17-18-08_EST-0500', '2017-01-01_17-29-00_EST-0500'],
        "bedroom1todoor2":      ['2017-01-01_16-14-22_EST-0500', '2017-01-01_17-19-09_EST-0500', '2017-01-01_17-29-57_EST-0500'],
    },
    
    "goal_marker_mx94": {
        "door1tolivroom1":      ['2017-01-01_17-46-44_EST-0500', '2017-01-01_18-02-38_EST-0500', '2017-01-01_18-26-09_EST-0500'],
        "livroom1tooffice1":    ['2017-01-01_17-49-28_EST-0500', '2017-01-01_18-04-21_EST-0500', '2017-01-01_18-28-01_EST-0500'],
        "office1tobathroom1":   ['2017-01-01_17-51-34_EST-0500', '2017-01-01_18-06-15_EST-0500'], # INCOMPLETE BAG: '2017-01-01_18-29-55_EST-0500'],
        "bathroom1tomusic1":    ['2017-01-01_17-53-59_EST-0500', '2017-01-01_18-08-06_EST-0500', '2017-01-01_18-32-01_EST-0500'],
        "music1tobedroom1":     ['2017-01-01_17-59-08_EST-0500', '2017-01-01_18-20-46_EST-0500', '2017-01-01_18-35-05_EST-0500'],
        "bedroom1todoor2":      ['2017-01-01_18-00-53_EST-0500', '2017-01-01_18-23-39_EST-0500', '2017-01-01_18-36-44_EST-0500'],
    },
    "goal_marker_mx70": {
        "door1tolivroom1":      ['2017-01-01_18-44-21_EST-0500', '2017-01-01_18-58-34_EST-0500', '2017-01-01_19-10-38_EST-0500'],
        "livroom1tooffice1":    ['2017-01-01_18-46-41_EST-0500', '2017-01-01_19-00-29_EST-0500', '2017-01-01_19-12-16_EST-0500'],
        "office1tobathroom1":   ['2017-01-01_18-48-51_EST-0500', '2017-01-01_19-02-52_EST-0500', '2017-01-01_19-14-11_EST-0500'],
        "bathroom1tomusic1":    ['2017-01-01_18-51-18_EST-0500', '2017-01-01_19-04-42_EST-0500', '2017-01-01_19-15-46_EST-0500'],
        "music1tobedroom1":     ['2017-01-01_18-54-04_EST-0500', '2017-01-01_19-06-43_EST-0500', '2017-01-01_19-18-42_EST-0500'],
        "bedroom1todoor2":      ['2017-01-01_18-56-07_EST-0500', '2017-01-01_19-08-51_EST-0500', '2017-01-01_19-22-31_EST-0500'],
    },
    
    "smooth_mx94_no_pois": {
        "door1tolivroom1":      ['2017-01-01_21-07-45_EST-0500', '2017-01-01_21-24-12_EST-0500', '2017-01-02_00-20-42_EST-0500'],
        "livroom1tooffice1":    ['2017-01-01_21-09-51_EST-0500', '2017-01-01_21-26-44_EST-0500', '2017-01-02_00-22-37_EST-0500'],
        "office1tobathroom1":   ['2017-01-01_21-15-17_EST-0500', '2017-01-01_21-28-45_EST-0500', '2017-01-02_00-24-25_EST-0500'],
        "bathroom1tomusic1":    ['2017-01-01_21-17-06_EST-0500', '2017-01-01_21-30-52_EST-0500', '2017-01-02_00-26-18_EST-0500'],
        "music1tobedroom1":     ['2017-01-01_21-19-13_EST-0500', '2017-01-01_21-32-56_EST-0500', '2017-01-02_00-28-29_EST-0500'],
        "bedroom1todoor2":      ['2017-01-01_21-21-18_EST-0500', '2017-01-01_21-34-55_EST-0500', '2017-01-02_00-30-01_EST-0500'],
    },
    "smooth_mx70_no_pois": {
        "door1tolivroom1":      ['2017-01-01_21-37-30_EST-0500', '2017-01-01_23-36-06_EST-0500', '2017-01-01_23-54-50_EST-0500'],
        "livroom1tooffice1":    ['2017-01-01_21-40-42_EST-0500', '2017-01-01_23-38-39_EST-0500', '2017-01-01_23-58-39_EST-0500'],
        "office1tobathroom1":   ['2017-01-01_21-44-21_EST-0500', '2017-01-01_23-41-15_EST-0500', '2017-01-02_00-00-40_EST-0500'],
        "bathroom1tomusic1":    ['2017-01-01_21-46-57_EST-0500', '2017-01-01_23-43-54_EST-0500', '2017-01-02_00-08-30_EST-0500'],
        "music1tobedroom1":     ['2017-01-01_23-29-16_EST-0500', '2017-01-01_23-47-33_EST-0500', '2017-01-02_00-13-07_EST-0500'],
        "bedroom1todoor2":      ['2017-01-01_23-32-32_EST-0500', '2017-01-01_23-51-26_EST-0500', '2017-01-02_00-16-53_EST-0500'],
    },
    
    "pos_div" : {
        "storage1to2" : {
            "nearcog_extremal__no_move":        ["2017-01-02_02-06-57_EST-0500", "2017-01-02_02-24-31_EST-0500"],
            "nearcog_extremal__cog2lopt":       ["2017-01-02_08-20-50_EST-0500", "2017-01-02_08-32-56_EST-0500"],
            "nearcog_extremal__nearcog_obst":   ["2017-01-02_08-50-45_EST-0500", "2017-01-02_09-07-48_EST-0500"],
            "altertile__no_move":               ["2017-01-02_09-20-26_EST-0500", "2017-01-02_09-32-59_EST-0500"],
            "altertile__cog2lopt":              ["2017-01-02_09-45-48_EST-0500", "2017-01-02_09-59-49_EST-0500"],
            "altertile__nearcog_obst":          ["2017-01-02_10-29-59_EST-0500", "2017-01-02_10-40-40_EST-0500"],
            "extredist__no_move":               ["2017-01-02_10-51-22_EST-0500", "2017-01-02_11-08-09_EST-0500"],
            "extredist__cog2lopt":              ["2017-01-02_11-40-14_EST-0500", "2017-01-02_11-59-34_EST-0500"],
            "extredist__nearcog_obst":          ["2017-01-02_12-11-18_EST-0500", "2017-01-02_12-31-47_EST-0500"]
        },
        "storage2to3" : {
            "nearcog_extremal__no_move":        ["2017-01-02_02-08-49_EST-0500", "2017-01-02_02-25-56_EST-0500"],
            "nearcog_extremal__cog2lopt":       ["2017-01-02_08-22-44_EST-0500", "2017-01-02_08-35-00_EST-0500"],
            "nearcog_extremal__nearcog_obst":   ["2017-01-02_08-58-57_EST-0500", "2017-01-02_09-09-07_EST-0500"],
            "altertile__no_move":               ["2017-01-02_09-22-03_EST-0500", "2017-01-02_09-35-51_EST-0500"],
            "altertile__cog2lopt":              ["2017-01-02_09-47-25_EST-0500", "2017-01-02_10-01-29_EST-0500"],
            "altertile__nearcog_obst":          ["2017-01-02_10-31-45_EST-0500", "2017-01-02_10-42-08_EST-0500"],
            "extredist__no_move":               ["2017-01-02_10-52-55_EST-0500", "2017-01-02_11-09-39_EST-0500"],
            "extredist__cog2lopt":              ["2017-01-02_11-41-51_EST-0500", "2017-01-02_12-02-03_EST-0500"],
            "extredist__nearcog_obst":          ["2017-01-02_12-12-49_EST-0500", "2017-01-02_12-32-56_EST-0500"]
        },
        "storage3tolivroom2" : {
            "nearcog_extremal__no_move":        ["2017-01-02_02-14-36_EST-0500", "2017-01-02_08-06-52_EST-0500"],
            "nearcog_extremal__cog2lopt":       ["2017-01-02_08-25-42_EST-0500", "2017-01-02_08-36-23_EST-0500"],
            "nearcog_extremal__nearcog_obst":   ["2017-01-02_09-00-40_EST-0500", "2017-01-02_09-10-30_EST-0500"],
            "altertile__no_move":               ["2017-01-02_09-23-47_EST-0500", "2017-01-02_09-37-15_EST-0500"],
            "altertile__cog2lopt":              ["2017-01-02_09-52-07_EST-0500", "2017-01-02_10-03-51_EST-0500"],
            "altertile__nearcog_obst":          ["2017-01-02_10-33-59_EST-0500", "2017-01-02_10-43-49_EST-0500"],
            "extredist__no_move":               ["2017-01-02_10-54-29_EST-0500", "2017-01-02_11-11-00_EST-0500"],
            "extredist__cog2lopt":              ["2017-01-02_11-52-24_EST-0500", "2017-01-02_12-03-34_EST-0500"],
            "extredist__nearcog_obst":          ["2017-01-02_12-14-14_EST-0500", "2017-01-02_12-34-12_EST-0500"]
        },
        "livroom2to1" : {
            "nearcog_extremal__no_move":        ["2017-01-02_02-19-02_EST-0500", "2017-01-02_08-10-09_EST-0500"],
            "nearcog_extremal__cog2lopt":       ["2017-01-02_08-28-40_EST-0500", "2017-01-02_08-40-09_EST-0500"],
            "nearcog_extremal__nearcog_obst":   ["2017-01-02_09-04-00_EST-0500", "2017-01-02_09-12-43_EST-0500"],
            "altertile__no_move":               ["2017-01-02_09-27-10_EST-0500", "2017-01-02_09-40-13_EST-0500"],
            "altertile__cog2lopt":              ["2017-01-02_09-54-57_EST-0500", "2017-01-02_10-05-41_EST-0500"],
            "altertile__nearcog_obst":          ["2017-01-02_10-35-50_EST-0500", "2017-01-02_10-45-33_EST-0500"],
                "extredist__no_move":             ["2017-01-02_10-58-15_EST-0500", "2017-01-02_11-13-45_EST-0500"],
            "extredist__cog2lopt":              ["2017-01-02_11-54-43_EST-0500", "2017-01-02_12-05-31_EST-0500"],
            "extredist__nearcog_obst":          ["2017-01-02_12-16-08_EST-0500", "2017-01-02_12-36-20_EST-0500"] 
        },
        "livroom1tokitchen1" : {
            "nearcog_extremal__no_move":        ["2017-01-02_02-20-53_EST-0500", "2017-01-02_08-12-32_EST-0500"],
            "nearcog_extremal__cog2lopt":       ["2017-01-02_08-29-55_EST-0500", "2017-01-02_08-41-33_EST-0500"],
            "nearcog_extremal__nearcog_obst":   ["2017-01-02_09-05-15_EST-0500", "2017-01-02_09-15-12_EST-0500"],
            "altertile__no_move":               ["2017-01-02_09-29-30_EST-0500", "2017-01-02_09-42-01_EST-0500"],
            "altertile__cog2lopt":              ["2017-01-02_09-57-35_EST-0500", "2017-01-02_10-07-50_EST-0500"],
            "altertile__nearcog_obst":          ["2017-01-02_10-37-17_EST-0500", "2017-01-02_10-47-38_EST-0500"], 
            "extredist__no_move":               ["2017-01-02_11-03-28_EST-0500", "2017-01-02_11-35-47_EST-0500"],
            "extredist__cog2lopt":              ["2017-01-02_11-56-37_EST-0500", "2017-01-02_12-07-39_EST-0500"],
            "extredist__nearcog_obst":          ["2017-01-02_12-17-58_EST-0500", "2017-01-02_12-38-16_EST-0500"] 
        }
    },
    
    "random_goals": {
        "0pathetic":    '2017-01-02_13-53-27_EST-0500',
        "1unknown":     '2017-01-02_14-10-07_EST-0500',
        "2incomplete":  '', 
        "3not_bad":     '2017-01-02_14-20-14_EST-0500',
        "4good":        '2017-01-02_14-33-02_EST-0500',
    }
}


def plot_all():
    gp = GroupPlot(bagDir, cacheDir, cwaveToolPath)
    
    gp.plotArray(bags["mx94_no_pois"]["door1tolivroom1"], "blue")
    gp.plotArray(bags["mx94_no_pois"]["livroom1tooffice1"], "blue")
    gp.plotArray(bags["mx94_no_pois"]["office1tobathroom1"], "blue")
    gp.plotArray(bags["mx94_no_pois"]["bathroom1tomusic1"], "blue")
    gp.plotArray(bags["mx94_no_pois"]["music1tobedroom1"], "blue")
    gp.plotArray(bags["mx94_no_pois"]["bedroom1todoor2"], "blue")
    
    
    gp.plotArray(bags["mx70_no_pois"]["door1tolivroom1"], "blue")
    gp.plotArray(bags["mx70_no_pois"]["livroom1tooffice1"], "blue")
    gp.plotArray(bags["mx70_no_pois"]["office1tobathroom1"], "blue")
    gp.plotArray(bags["mx70_no_pois"]["bathroom1tomusic1"], "blue")
    gp.plotArray(bags["mx70_no_pois"]["music1tobedroom1"], "blue")
    gp.plotArray(bags["mx70_no_pois"]["bedroom1todoor2"], "blue")
    
    
    gp.plotArray(bags["mx94_pois"]["door1tolivroom1"], "blue")
    gp.plotArray(bags["mx94_pois"]["livroom1tooffice1"], "blue")
    gp.plotArray(bags["mx94_pois"]["office1tobathroom1"], "blue")
    gp.plotArray(bags["mx94_pois"]["bathroom1tomusic1"], "blue")
    gp.plotArray(bags["mx94_pois"]["music1tobedroom1"], "blue")
    gp.plotArray(bags["mx94_pois"]["bedroom1todoor2"], "blue")
    
    
    gp.plotArray(bags["mx70_pois"]["door1tolivroom1"], "blue")
    gp.plotArray(bags["mx70_pois"]["livroom1tooffice1"], "blue")
    gp.plotArray(bags["mx70_pois"]["office1tobathroom1"], "blue")
    gp.plotArray(bags["mx70_pois"]["bathroom1tomusic1"], "blue")
    gp.plotArray(bags["mx70_pois"]["music1tobedroom1"], "blue")
    gp.plotArray(bags["mx70_pois"]["bedroom1todoor2"], "blue")
    
    


    gp.plotArray(bags["two_mx_no_pois"]["door1tolivroom1"], "blue")
    gp.plotArray(bags["two_mx_no_pois"]["livroom1tooffice1"], "blue")
    gp.plotArray(bags["two_mx_no_pois"]["office1tobathroom1"], "blue")
    gp.plotArray(bags["two_mx_no_pois"]["bathroom1tomusic1"], "blue")
    gp.plotArray(bags["two_mx_no_pois"]["music1tobedroom1"], "blue")
    gp.plotArray(bags["two_mx_no_pois"]["bedroom1todoor2"], "blue")
    
    
    gp.plotArray(bags["two_mx_with_pois"]["door1tolivroom1"], "blue")
    gp.plotArray(bags["two_mx_with_pois"]["livroom1tooffice1"], "blue")
    gp.plotArray(bags["two_mx_with_pois"]["office1tobathroom1"], "blue")
    gp.plotArray(bags["two_mx_with_pois"]["bathroom1tomusic1"], "blue")
    gp.plotArray(bags["two_mx_with_pois"]["music1tobedroom1"], "blue")
    gp.plotArray(bags["two_mx_with_pois"]["bedroom1todoor2"], "blue")
    
    
    gp.plotArray(bags["goal_marker_mx94"]["door1tolivroom1"], "blue")
    gp.plotArray(bags["goal_marker_mx94"]["livroom1tooffice1"], "blue")
    gp.plotArray(bags["goal_marker_mx94"]["office1tobathroom1"], "blue")
    gp.plotArray(bags["goal_marker_mx94"]["bathroom1tomusic1"], "blue")
    gp.plotArray(bags["goal_marker_mx94"]["music1tobedroom1"], "blue")
    gp.plotArray(bags["goal_marker_mx94"]["bedroom1todoor2"], "blue")
    
    
    gp.plotArray(bags["goal_marker_mx70"]["door1tolivroom1"], "blue")
    gp.plotArray(bags["goal_marker_mx70"]["livroom1tooffice1"], "blue")
    gp.plotArray(bags["goal_marker_mx70"]["office1tobathroom1"], "blue")
    gp.plotArray(bags["goal_marker_mx70"]["bathroom1tomusic1"], "blue")
    gp.plotArray(bags["goal_marker_mx70"]["music1tobedroom1"], "blue")
    gp.plotArray(bags["goal_marker_mx70"]["bedroom1todoor2"], "blue")
    
    
    
    
    
    gp.plotArray(bags["smooth_mx94_no_pois"]["door1tolivroom1"], "blue")
    gp.plotArray(bags["smooth_mx94_no_pois"]["livroom1tooffice1"], "blue")
    gp.plotArray(bags["smooth_mx94_no_pois"]["office1tobathroom1"], "blue")
    gp.plotArray(bags["smooth_mx94_no_pois"]["bathroom1tomusic1"], "blue")
    gp.plotArray(bags["smooth_mx94_no_pois"]["music1tobedroom1"], "blue")
    gp.plotArray(bags["smooth_mx94_no_pois"]["bedroom1todoor2"], "blue")
    
    
    gp.plotArray(bags["smooth_mx70_no_pois"]["door1tolivroom1"], "blue")
    gp.plotArray(bags["smooth_mx70_no_pois"]["livroom1tooffice1"], "blue")
    gp.plotArray(bags["smooth_mx70_no_pois"]["office1tobathroom1"], "blue")
    gp.plotArray(bags["smooth_mx70_no_pois"]["bathroom1tomusic1"], "blue")
    gp.plotArray(bags["smooth_mx70_no_pois"]["music1tobedroom1"], "blue")
    gp.plotArray(bags["smooth_mx70_no_pois"]["bedroom1todoor2"], "blue")




    gp.plotArray(bags["change_of_mind_mx85_no_pois"]["kitchen1storage1"], "blue")
    gp.plotArray(bags["change_of_mind_mx85_no_pois"]["storage1tostorage3"], "blue")
    gp.plotArray(bags["change_of_mind_mx85_with_pois"]["kitchen1storage1"], "blue")
    gp.plotArray(bags["change_of_mind_mx85_with_pois"]["storage1tostorage3"], "blue")

    gp.plotArray(bags["nav_to_non_poi_mx94"]["kitchen1tooffice1"], "blue")
    gp.plotArray(bags["nav_to_non_poi_mx94"]["office1tobedroom1"], "blue")
    gp.plotArray(bags["nav_to_non_poi_mx70"]["kitchen1tooffice1"], "blue")
    gp.plotArray(bags["nav_to_non_poi_mx70"]["office1tobedroom1"], "blue")
    




    gp.plotArray(bags["pos_div"]["storage1to2"]["nearcog_extremal__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["storage1to2"]["nearcog_extremal__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["storage1to2"]["nearcog_extremal__nearcog_obst"], "blue")
    gp.plotArray(bags["pos_div"]["storage1to2"]["altertile__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["storage1to2"]["altertile__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["storage1to2"]["altertile__nearcog_obst"], "blue")
    gp.plotArray(bags["pos_div"]["storage1to2"]["extredist__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["storage1to2"]["extredist__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["storage1to2"]["extredist__nearcog_obst"], "blue")

    gp.plotArray(bags["pos_div"]["storage2to3"]["nearcog_extremal__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["storage2to3"]["nearcog_extremal__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["storage2to3"]["nearcog_extremal__nearcog_obst"], "blue")
    gp.plotArray(bags["pos_div"]["storage2to3"]["altertile__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["storage2to3"]["altertile__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["storage2to3"]["altertile__nearcog_obst"], "blue")
    gp.plotArray(bags["pos_div"]["storage2to3"]["extredist__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["storage2to3"]["extredist__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["storage2to3"]["extredist__nearcog_obst"], "blue")

    gp.plotArray(bags["pos_div"]["storage3tolivroom2"]["nearcog_extremal__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["storage3tolivroom2"]["nearcog_extremal__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["storage3tolivroom2"]["nearcog_extremal__nearcog_obst"], "blue")
    gp.plotArray(bags["pos_div"]["storage3tolivroom2"]["altertile__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["storage3tolivroom2"]["altertile__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["storage3tolivroom2"]["altertile__nearcog_obst"], "blue")
    gp.plotArray(bags["pos_div"]["storage3tolivroom2"]["extredist__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["storage3tolivroom2"]["extredist__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["storage3tolivroom2"]["extredist__nearcog_obst"], "blue")
    
    gp.plotArray(bags["pos_div"]["livroom2to1"]["nearcog_extremal__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["livroom2to1"]["nearcog_extremal__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["livroom2to1"]["nearcog_extremal__nearcog_obst"], "blue")
    gp.plotArray(bags["pos_div"]["livroom2to1"]["altertile__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["livroom2to1"]["altertile__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["livroom2to1"]["altertile__nearcog_obst"], "blue")
    gp.plotArray(bags["pos_div"]["livroom2to1"]["extredist__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["livroom2to1"]["extredist__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["livroom2to1"]["extredist__nearcog_obst"], "blue")

    gp.plotArray(bags["pos_div"]["livroom1tokitchen1"]["nearcog_extremal__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["livroom1tokitchen1"]["nearcog_extremal__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["livroom1tokitchen1"]["nearcog_extremal__nearcog_obst"], "blue")
    gp.plotArray(bags["pos_div"]["livroom1tokitchen1"]["altertile__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["livroom1tokitchen1"]["altertile__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["livroom1tokitchen1"]["altertile__nearcog_obst"], "blue")
    gp.plotArray(bags["pos_div"]["livroom1tokitchen1"]["extredist__no_move"], "blue")
    gp.plotArray(bags["pos_div"]["livroom1tokitchen1"]["extredist__cog2lopt"], "blue")
    gp.plotArray(bags["pos_div"]["livroom1tokitchen1"]["extredist__nearcog_obst"], "blue")

    gp.show()


def plot_mx94_vs_mx70_no_pois2():
    dbase = RecordByStamp(bagDir, cacheDir, cwaveToolPath)
    
    fig = plt.figure(facecolor='white')
    plt.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.1)
    
    #ax_map   = plt.subplot(121)
    #ax_dist1 = plt.subplot(222)
    #ax_entr1 = plt.subplot(224, sharex=ax_dist1)

    #ax_map   = plt.subplot(131)
    #ax_dist1 = plt.subplot(232)
    #ax_entr1 = plt.subplot(235, sharex=ax_dist1)
    #ax_dist2 = plt.subplot(233)
    #ax_entr2 = plt.subplot(236, sharex=ax_dist2)
    ##ax_dist3 = plt.subplot(244)
    ##ax_entr3 = plt.subplot(248, sharex=ax_dist3)
    
    ax_map   = plt.subplot(141)
    ax_dist1 = plt.subplot(242)
    ax_entr1 = plt.subplot(246, sharex=ax_dist1)
    ax_dist2 = plt.subplot(243)
    ax_entr2 = plt.subplot(247, sharex=ax_dist2)
    ax_dist3 = plt.subplot(244)
    ax_entr3 = plt.subplot(248, sharex=ax_dist3)
    
    PlottableBagRecord(dbase.getRecord(bags["mx94_no_pois"]["door1tolivroom1"][0])).plotMaps(ax_map)

    PlottableBagRecord(dbase.getRecord(bags["mx94_no_pois"]["door1tolivroom1"][0])).plotPathDistEntropy(ax_map, ax_dist1, ax_entr1, "#9999FF")
    PlottableBagRecord(dbase.getRecord(bags["mx94_no_pois"]["door1tolivroom1"][1])).plotPathDistEntropy(ax_map, ax_dist1, ax_entr1, "#3333FF")
    PlottableBagRecord(dbase.getRecord(bags["mx94_no_pois"]["door1tolivroom1"][2])).plotPathDistEntropy(ax_map, ax_dist1, ax_entr1, "#000099")
    PlottableBagRecord(dbase.getRecord(bags["mx70_no_pois"]["door1tolivroom1"][0])).plotPathDistEntropy(ax_map, ax_dist1, ax_entr1, "#FF9999")
    PlottableBagRecord(dbase.getRecord(bags["mx70_no_pois"]["door1tolivroom1"][1])).plotPathDistEntropy(ax_map, ax_dist1, ax_entr1, "#FF3333")
    PlottableBagRecord(dbase.getRecord(bags["mx70_no_pois"]["door1tolivroom1"][2])).plotPathDistEntropy(ax_map, ax_dist1, ax_entr1, "#BB0000")
    
    
    PlottableBagRecord(dbase.getRecord(bags["mx94_no_pois"]["office1tobathroom1"][0])).plotPathDistEntropy(ax_map, ax_dist2, ax_entr2, "#9999FF")
    PlottableBagRecord(dbase.getRecord(bags["mx94_no_pois"]["office1tobathroom1"][1])).plotPathDistEntropy(ax_map, ax_dist2, ax_entr2, "#3333FF")
    PlottableBagRecord(dbase.getRecord(bags["mx94_no_pois"]["office1tobathroom1"][2])).plotPathDistEntropy(ax_map, ax_dist2, ax_entr2, "#000099")
    PlottableBagRecord(dbase.getRecord(bags["mx70_no_pois"]["office1tobathroom1"][0])).plotPathDistEntropy(ax_map, ax_dist2, ax_entr2, "#FF9999")
    PlottableBagRecord(dbase.getRecord(bags["mx70_no_pois"]["office1tobathroom1"][1])).plotPathDistEntropy(ax_map, ax_dist2, ax_entr2, "#FF3333")
    PlottableBagRecord(dbase.getRecord(bags["mx70_no_pois"]["office1tobathroom1"][2])).plotPathDistEntropy(ax_map, ax_dist2, ax_entr2, "#BB0000")
    
    
    PlottableBagRecord(dbase.getRecord(bags["mx94_no_pois"]["music1tobedroom1"][0])).plotPathDistEntropy(ax_map, ax_dist3, ax_entr3, "#9999FF")
    PlottableBagRecord(dbase.getRecord(bags["mx94_no_pois"]["music1tobedroom1"][1])).plotPathDistEntropy(ax_map, ax_dist3, ax_entr3, "#3333FF")
    PlottableBagRecord(dbase.getRecord(bags["mx94_no_pois"]["music1tobedroom1"][2])).plotPathDistEntropy(ax_map, ax_dist3, ax_entr3, "#000099")
    PlottableBagRecord(dbase.getRecord(bags["mx70_no_pois"]["music1tobedroom1"][0])).plotPathDistEntropy(ax_map, ax_dist3, ax_entr3, "#FF9999")
    PlottableBagRecord(dbase.getRecord(bags["mx70_no_pois"]["music1tobedroom1"][1])).plotPathDistEntropy(ax_map, ax_dist3, ax_entr3, "#FF3333")
    PlottableBagRecord(dbase.getRecord(bags["mx70_no_pois"]["music1tobedroom1"][2])).plotPathDistEntropy(ax_map, ax_dist3, ax_entr3, "#BB0000")
    
    #plt.tight_layout() 
    plt.show()()



def drawFamilyOfBagRecords(ax_map, ax_dist, ax_entr, bag_stamps, colors):
    for k,stamp in enumerate(bag_stamps):
        bagRecord = DBASE.getRecord(stamp)
        pbr = PlottableBagRecord(bagRecord)
        pbr.plotPathDistEntropy(ax_map, ax_dist, ax_entr, colors[k])

def plot_compare2_6path_2map(stamp_set1, stamp_set2):
    fig = plt.figure(facecolor='white')
    plt.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.1)
    
    ax_map1   = plt.subplot(2,4,1)
    
    ax_dist1 = plt.subplot(4,4,2)
    ax_entr1 = plt.subplot(4,4,6, sharex=ax_dist1)
    ax_dist2 = plt.subplot(4,4,3)
    ax_entr2 = plt.subplot(4,4,7, sharex=ax_dist2)
    ax_dist3 = plt.subplot(4,4,4)
    ax_entr3 = plt.subplot(4,4,8, sharex=ax_dist3)
    
    ax_map2   = plt.subplot(2,4,5)
    
    ax_dist4 = plt.subplot(4,4,10)
    ax_entr4 = plt.subplot(4,4,14, sharex=ax_dist4)
    ax_dist5 = plt.subplot(4,4,11)
    ax_entr5 = plt.subplot(4,4,15, sharex=ax_dist5)
    ax_dist6 = plt.subplot(4,4,12)
    ax_entr6 = plt.subplot(4,4,16, sharex=ax_dist6)
    
    color_set1 = ["#9999FF","#3333FF","#000099"]
    color_set2 = ["#FF9999","#FF3333","#BB0000"]
    PlottableBagRecord(DBASE.getRecord(stamp_set1["door1tolivroom1"][0])).plotMaps(ax_map1)

    drawFamilyOfBagRecords(ax_map1, ax_dist1, ax_entr1, stamp_set1["door1tolivroom1"],    color_set1)
    drawFamilyOfBagRecords(ax_map1, ax_dist1, ax_entr1, stamp_set2["door1tolivroom1"],    color_set2)
    drawFamilyOfBagRecords(ax_map1, ax_dist2, ax_entr2, stamp_set1["office1tobathroom1"], color_set1)
    drawFamilyOfBagRecords(ax_map1, ax_dist2, ax_entr2, stamp_set2["office1tobathroom1"], color_set2)
    drawFamilyOfBagRecords(ax_map1, ax_dist3, ax_entr3, stamp_set1["music1tobedroom1"],   color_set1)
    drawFamilyOfBagRecords(ax_map1, ax_dist3, ax_entr3, stamp_set2["music1tobedroom1"],   color_set2)
    
    
    PlottableBagRecord(DBASE.getRecord(stamp_set1["door1tolivroom1"][0])).plotMaps(ax_map2)

    drawFamilyOfBagRecords(ax_map2, ax_dist4, ax_entr4, stamp_set1["livroom1tooffice1"], color_set1)
    drawFamilyOfBagRecords(ax_map2, ax_dist4, ax_entr4, stamp_set2["livroom1tooffice1"], color_set2)
    drawFamilyOfBagRecords(ax_map2, ax_dist5, ax_entr5, stamp_set1["bathroom1tomusic1"], color_set1)
    drawFamilyOfBagRecords(ax_map2, ax_dist5, ax_entr5, stamp_set2["bathroom1tomusic1"], color_set2)
    drawFamilyOfBagRecords(ax_map2, ax_dist6, ax_entr6, stamp_set1["bedroom1todoor2"],   color_set1)
    drawFamilyOfBagRecords(ax_map2, ax_dist6, ax_entr6, stamp_set2["bedroom1todoor2"],   color_set2)

    #plt.tight_layout() 
    plt.show()()




def plot_compare2_6path_3map(stamp_set1, stamp_set2):
    fig = plt.figure(facecolor='white')
    plt.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.1)
    
    ax_map1   = plt.subplot(3,3,1)
    ax_dist1 = plt.subplot(6,3,2)
    ax_dist2 = plt.subplot(6,3,3)
    ax_entr1 = plt.subplot(6,3,5, sharex=ax_dist1)
    ax_entr2 = plt.subplot(6,3,6, sharex=ax_dist2)
    
    ax_map2   = plt.subplot(3,3,4)
    ax_dist3 = plt.subplot(6,3,8)
    ax_dist4 = plt.subplot(6,3,9)
    ax_entr3 = plt.subplot(6,3,11, sharex=ax_dist3)    
    ax_entr4 = plt.subplot(6,3,12, sharex=ax_dist4)
    
    ax_map3   = plt.subplot(3,3,7)
    ax_dist5 = plt.subplot(6,3,14)
    ax_dist6 = plt.subplot(6,3,15)
    ax_entr5 = plt.subplot(6,3,17, sharex=ax_dist5)
    ax_entr6 = plt.subplot(6,3,18, sharex=ax_dist6)
    
    
    color_set1 = ["#9999FF","#3333FF","#000099"]
    color_set2 = ["#FF9999","#FF3333","#BB0000"]
    
    
        #drawFamilyOfBagRecords(ax_map1, ax_dist2, ax_entr2, stamp_set1["office1tobathroom1"], color_set1)
    #drawFamilyOfBagRecords(ax_map1, ax_dist2, ax_entr2, stamp_set2["office1tobathroom1"], color_set2)
    #drawFamilyOfBagRecords(ax_map1, ax_dist3, ax_entr3, stamp_set1["music1tobedroom1"],   color_set1)
    #drawFamilyOfBagRecords(ax_map1, ax_dist3, ax_entr3, stamp_set2["music1tobedroom1"],   color_set2)
    
    #PlottableBagRecord(DBASE.getRecord(stamp_set1["door1tolivroom1"][0])).plotMaps(ax_map2)
    #drawFamilyOfBagRecords(ax_map2, ax_dist4, ax_entr4, stamp_set1["livroom1tooffice1"], color_set1)
    #drawFamilyOfBagRecords(ax_map2, ax_dist4, ax_entr4, stamp_set2["livroom1tooffice1"], color_set2)
    #drawFamilyOfBagRecords(ax_map2, ax_dist5, ax_entr5, stamp_set1["bathroom1tomusic1"], color_set1)
    #drawFamilyOfBagRecords(ax_map2, ax_dist5, ax_entr5, stamp_set2["bathroom1tomusic1"], color_set2)
    #drawFamilyOfBagRecords(ax_map2, ax_dist6, ax_entr6, stamp_set1["bedroom1todoor2"],   color_set1)
    #drawFamilyOfBagRecords(ax_map2, ax_dist6, ax_entr6, stamp_set2["bedroom1todoor2"],   color_set2)

    #PlottableBagRecord(DBASE.getRecord(stamp_set1["door1tolivroom1"][0])).plotMaps(ax_map3)
    #drawFamilyOfBagRecords(ax_map2, ax_dist4, ax_entr4, stamp_set1["livroom1tooffice1"], color_set1)
    #drawFamilyOfBagRecords(ax_map2, ax_dist4, ax_entr4, stamp_set2["livroom1tooffice1"], color_set2)
    #drawFamilyOfBagRecords(ax_map2, ax_dist5, ax_entr5, stamp_set1["bathroom1tomusic1"], color_set1)
    #drawFamilyOfBagRecords(ax_map2, ax_dist5, ax_entr5, stamp_set2["bathroom1tomusic1"], color_set2)
    #drawFamilyOfBagRecords(ax_map2, ax_dist6, ax_entr6, stamp_set1["bedroom1todoor2"],   color_set1)
    #drawFamilyOfBagRecords(ax_map2, ax_dist6, ax_entr6, stamp_set2["bedroom1todoor2"],   color_set2)
    
    PlottableBagRecord(DBASE.getRecord(stamp_set1["door1tolivroom1"][0])).plotMaps(ax_map1)
    drawFamilyOfBagRecords(ax_map1, ax_dist1, ax_entr1, stamp_set1["door1tolivroom1"],   color_set1)
    drawFamilyOfBagRecords(ax_map1, ax_dist1, ax_entr1, stamp_set2["door1tolivroom1"],   color_set2)
    drawFamilyOfBagRecords(ax_map1, ax_dist2, ax_entr2, stamp_set1["bathroom1tomusic1"], color_set1)
    drawFamilyOfBagRecords(ax_map1, ax_dist2, ax_entr2, stamp_set2["bathroom1tomusic1"], color_set2)
    
    PlottableBagRecord(DBASE.getRecord(stamp_set1["door1tolivroom1"][0])).plotMaps(ax_map2)
    drawFamilyOfBagRecords(ax_map2, ax_dist3, ax_entr3, stamp_set1["livroom1tooffice1"],   color_set1)
    drawFamilyOfBagRecords(ax_map2, ax_dist3, ax_entr3, stamp_set2["livroom1tooffice1"],   color_set2)
    drawFamilyOfBagRecords(ax_map2, ax_dist4, ax_entr4, stamp_set1["bedroom1todoor2"], color_set1)
    drawFamilyOfBagRecords(ax_map2, ax_dist4, ax_entr4, stamp_set2["bedroom1todoor2"], color_set2)
    
    PlottableBagRecord(DBASE.getRecord(stamp_set1["door1tolivroom1"][0])).plotMaps(ax_map3)
    drawFamilyOfBagRecords(ax_map3, ax_dist5, ax_entr5, stamp_set1["music1tobedroom1"], color_set1)
    drawFamilyOfBagRecords(ax_map3, ax_dist5, ax_entr5, stamp_set2["music1tobedroom1"], color_set2)
    drawFamilyOfBagRecords(ax_map3, ax_dist6, ax_entr6, stamp_set1["office1tobathroom1"],   color_set1)
    drawFamilyOfBagRecords(ax_map3, ax_dist6, ax_entr6, stamp_set2["office1tobathroom1"],   color_set2)

    #plt.tight_layout() 
    plt.show()()


if __name__=="__main__":
    #plot_all()
    plot_compare2_6path_3map(bags["mx94_no_pois"], bags["mx70_no_pois"])
