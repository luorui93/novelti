#!/usr/bin/env python

from CommandVariator import *

cmds = [("roslaunch lthmi_nav exp.launch  "
        "tries:=%(tries)d  map:=%(map)s  resol:=%(resol)f  path:=%(path)d  "
        "mx:=%(mx)s  period:=%(period)f  vel:=%(vel)f  trobot:=%(trobot)f  "
        "phigh:=%(phigh)f  plow:=%(plow)f  peps:=%(peps)e  "
        "pos:=%(pos)s  ksafe:=%(ksafe)f  "
        "div:=%(div)s  popt:=%(popt)s  "
        "rviz:=%(rviz)s  bag:=%(bag)d  "
        "bagpath:=%(dir)s/bag/%(bagid)s.bag  ")
        ,
        "rm %(dir)s/bag/%(bagid)s.bag.success"
        ,
        "roslaunch lthmi_nav bag_processor.launch bag:=%(dir)s/bag/%(bagid)s.bag out:=%(dir)s/stats/%(bagid)s.bag.txt"
        ,
        "rm  %(dir)s/bag/%(bagid)s.bag.stats.success"
        ,
        "tar -czf %(dir)s/tar/%(bagid)s.bag.tar.gz -C %(dir)s/bag  %(bagid)s.bag"
        ,
        "rm -rf %(dir)s/bag/%(bagid)s.bag"
        ]

varprms = {
    'dir' :"/home/sd/Desktop/lthmi_nav_data",
    'bagid': lambda: "lthmi-auto-nav-experiment-%s" % (datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")),
    'tries': 30,
    'map': 'ak500inflated',
    'resol': 0.1,
    'path': [7, 12, 59],
    'mx': ['mx31','mx40','mx49','mx55','mx61','mx70','mx79','mx85','mx91','mx100'],
    'period': 1.0,
    'vel': 3.0,
    'trobot' : 0.01,
    
    'phigh': 0.95,
    'plow': 0.6,
    'peps': 1.0e-12,
    
    'pos': ["no_move", "ra_maxprob", "maxprob_obst", "nearcog_obst", "cog2lopt", "ramaxprob2lopt", "maxprob2lopt", "gopt"],
    'ksafe': 0.90,
    
    'div': ["vtile", "htile", "altertile", "equidist", "extremal", "extredist"],
    'popt': 'equal',
    
    'rviz': 'static',
    'bag': 1,
}

prmsets = {
    'set0' : {
        'tries': 1,
        'mx' : ['mx91', 'mx70'],
        'pos': 'cog2lopt',
        'div': 'vtile',
        'path': 2,
        'vel': 4.0,
        'period': 0.5
    },
    'set1' : {
        'tries': 3,
        'mx' : ['mx91', 'mx70'],
        'pos': ['maxprob_obst', 'nearcog_obst'],
        'div': ['vtile', 'htile'],
        'path': [1,2],
        'vel': 3.0,
        'period': 1.0
    },
    'set2' : {
        'tries': 10,
        'mx' : ['mx70', 'mx79', 'mx91'],
        'pos': ['maxprob_obst', 'nearcog_obst', 'cog2lopt'],
        'div': ['vtile', 'htile'],
        'path': [100,103,110,116],
        'vel': 2.0,
        'period': 1.0
    },
    'fast2' : {
        'tries': 5,
        'mx' : ['mx91'],
        'pos': ['maxprob_obst', 'cog2lopt', 'cog2gopt'],
        'div': ['equidist', 'htile'],
        'path': [100,110],
        'vel': 8.0,
        'period': 0.25
    },
    'qqq' : {
        'tries': 20,
        'mx' : ['mx70', 'mx79', 'mx91', 'mx100'],
        'pos': ['maxprob_obst', 'nearcog_obst', 'cog2lopt'],
        'div': ['vtile', 'htile'],
        'path': [100,103,110,116],
        'vel': 8.0,
        'period': 0.25
    },
    'qqq_eq' : {
        'tries': 7,
        'mx' : ['mx70', 'mx91', 'mx100', 'mx79'],
        'pos': ['maxprob_obst', 'nearcog_obst', 'cog2lopt'],
        'div': ['equidist'],
        'path': [100,103,116,110],
        'vel': 8.0,
        'period': 0.25
    },
    'test_all' : {
        'tries': 3,
        'mx' : ['mx70', 'mx91', 'mx100', 'mx79'],
        'path': [100,103,116,110],
        'vel': 1.0,
        'period': 1.0
    }
}

import sys

usage = """ USAGE:
    ./runexps.py <prmset1> [<prmset2>.... [<prmsetN>]]

"""
if __name__=="__main__":
    if len(sys.argv)<2:
        print usage
        sys.exit(1)
    for s in sys.argv[1:]:
        if s not in prmsets.keys():
            print "ERROR: non-existing parameter set '%s'" % s
            print usage
            sys.exit(1)
        prmset = dict(varprms) #copy
        prmset.update(prmsets[s])
        print "======================================================== RUNNING paramset=%s ========================================================\n prms=%s" % (s, str(prmset))
        c = CommandVariator(cmds, prmset)
        c.run()
        print "======================================================== SUCCESSFULLY FINISHED paramset=%s ========================================================\n prms=%s" % (s, str(prmset))
