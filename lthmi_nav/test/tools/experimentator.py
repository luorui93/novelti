#!/usr/bin/env python

import datetime

cmd_templates = [("roslaunch lthmi_nav run.launch  "
        "tries:=%(tries)d  map:=%(map)s  resol:=%(resol)f  path:=%(path)d  "
        "mx:=%(mx)s  period:=%(period)f  vel:=%(vel)f  trobot:=%(trobot)f  "
        "phigh:=%(phigh)f  plow:=%(plow)f  peps:=%(peps)e  "
        "pos:=%(pos)s  ksafe:=%(ksafe)f  "
        "div:=%(div)s  popt:=%(popt)s  "
        "rviz:=%(rviz)s  bag:=%(bag)d  "
        "bagpath:=%(dir)s/bag/%(bagid)s.bag  ")
        ,
        "roslaunch lthmi_nav bag_processor.launch bag:=%(dir)s/bag/%(bagid)s.bag out:=%(dir)s/stats/%(bagid)s.bag.txt"
        ,
        "tar -czf %(dir)s/tar/%(bagid)s.bag.tar.gz %(dir)s/bag/%(bagid)s.bag"
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
    
    'pos': ['maxprob_obst', 'cog2lopt'], #ra_maxprob  maxprob_euq  maxprob_obst  cog_euq  nearcog_euq  nearcog_obst  cog2lopt  cog2gopt
    'ksafe': 0.9,
    
    'div': ['vtile', 'equidist'], #vtile  htile  equidist  extremal
    'popt': 'equal',
    
    'rviz': 'none',
    'bag': 0,
}

class Variator(object):
    def __init__(self, arr):
        self.arr = arr
        self.ret = list(arr)
        self.first = True

    def __iter__(self):
        return self

    # Python 3 compatibility
    def __next__(self):
        return self.next()

    def next(self):
        if self.first:
            self.first = False
            return self.ret
        else:
            firstNonZero = (i for i,v in enumerate(self.ret) if v>0).next()
            self.ret[firstNonZero] -= 1
            for k in range(firstNonZero):
                self.ret[k] = self.arr[k]
            return self.ret

class ParameterVariator:
    def __init__(self, prms):
        self.prms = prms
        self.ret = dict.fromkeys(prms.keys(), None)
        arr = []
        self.keys = []
        self.variatedKeyIds = []
        for k, v in self.prms.iteritems():
            if type(v)==list:
                self.variatedKeyIds.append(len(self.keys))
                arr.append(len(v)-1)
            elif not callable(v):
                self.ret[k] = v
            self.keys.append(k)
        self.variator = Variator(arr)
        
    def __iter__(self):
        return self

    # Python 3 compatibility
    def __next__(self):
        return self.next()

    def next(self):
        ids = self.variator.next()
        for k,v in enumerate(ids):
            pkey = self.keys[self.variatedKeyIds[k]]
            if callable(self.prms[pkey]):
                self.ret[pkey] = self.prms[pkey]()
            else:
                self.ret[pkey] = self.prms[pkey][v]
        for k,v in self.prms.iteritems():
            if callable(v):
                self.ret[k] = v()
        return self.ret

class CommandVariator:
    def __init__(self, cmds, varprms):
        self.prmVariator = ParameterVariator(varprms)
        self.cmds = cmds
        
    def run(self):
        for prmset in self.prmVariator:
            for tmpl in self.cmds:
                cmd = tmpl % prmset
                print "Invoking command: \n$ %s" % cmd
                status = 1 #subprocess.call(cmd, shell=True)
                if status != 0:
                    raise RuntimeError("Command '%s' FAILED (exit status=%d)" % (cmd,status))
            
            
if __name__=="__main__":
    c = CommandVariator(cmd_templates, varprms)
    c.run()

