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
        "ls -l %(dir)s/bag/%(bagid)s.bag"
        ,
        "roslaunch lthmi_nav bag_processor.launch bag:=%(dir)s/bag/%(bagid)s.bag out:=%(dir)s/stats/%(bagid)s.bag.txt"
        ,
        "ls -l %(dir)s/stats/%(bagid)s.bag.txt"
        ,
        "tar -czf %(dir)s/tar/%(bagid)s.bag.tar.gz -C %(dir)s/bag  %(bagid)s.bag"
        ,
        "rm -rf %(dir)s/bag/%(bagid)s.bag"
        ]

varprms = {
    'dir' :"/home/sd/Desktop/lthmi_nav_data",
    'bagid': lambda: "lthmi-auto-nav-experiment-%s" % (datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")),
    'tries': 30,
    'map': 'ak500inflated',
    'resol': 0.1,
    'path': [7, 12, 59],
    
    'mx': ['mx55', 'mx85'],
    'period': 1.0,
    'vel': 3.0,
    'trobot' : 0.025,
    
    'phigh': 0.9,
    'plow': 0.6,
    'peps': 1.0e-12,
    
    'pos': ['ra_maxprob', 'maxprob_euq', 'maxprob_obst', 'cog_euq', 'nearcog_euq', 'nearcog_obst', 'cog2lopt', 'cog2gopt'],
    'ksafe': 0.95,
    
    'div': ['vtile', 'htile', 'equidist', 'extremal'],
    'popt': 'equal',
    
    'rviz': 'static',
    'bag': 1,
}

prmsets = {
    'set0' : {
        'mx' : ['mx91', 'mx70']
        'pos': 'cog2lopt',
        'div': 'vtile',
        'path': 2,
        'tries': 1,
        'vel': 4.0,
        'period': 0.5
    },
    'set1' : {
        'pos': ['maxprob_obst', 'nearcog_obst'],
        'div': ['vtile', 'htile'],
        'path': [1,2,7],
        'tries': 3,
        'vel': 2.0,
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
        print "======================= RUNNING paramset=%s =======================\n prms=%s" % (s, str(prmset))
        c = CommandVariator(cmds, prmset)
        c.run()

