#!/usr/bin/env python

import datetime
import subprocess

class Variator(object):
    def __init__(self, arr):
        self.arr = arr
        self.ret = list(arr)
        self.first = True
        self.total = 1
        for n in arr:
            self.total *= (n+1)
        self.ind = 0

    def __iter__(self):
        return self

    # Python 3 compatibility
    def __next__(self):
        return self.next()

    def next(self):
        self.ind += 1
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
                print "Invoking command (variation %d out of %d): \n$ %s" % \
                    (self.prmVariator.variator.ind, self.prmVariator.variator.total, cmd)
                status = subprocess.call(cmd, shell=True)
                if status != 0:
                    raise RuntimeError("Command '%s' FAILED (exit status=%d)" % (cmd,status))
            
            
if __name__=="__main__":
    c = CommandVariator(cmd_templates, varprms)
    c.run()

