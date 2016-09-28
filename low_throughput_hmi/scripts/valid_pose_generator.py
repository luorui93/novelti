#!/usr/bin/env python

import sys
import re
from random import randint

"""
How to use:
    $ cat ../map/office2_1.map | ../scripts/valid_pose_generator.py exp <number of experiments> <map file name> <resolution> <tries> > ../exp/exp_set_2.yaml

    Example:
        $ cat ../map/office2_1.map | ../scripts/valid_pose_generator.py exp 30 office2_1.map 0.1 3 > ../exp/exp_set_3/experiments.yaml

"""
    
class RandomPoseGenerator:
    def __init__(self, src):
        self.data = self.readMap(src)
        self.width = len(self.data)
        self.height = len(self.data[0])
    
    def is_free(self, x, y):
        #print "try (%d,%d)=>%d" % (x,y, self.data[x][y])
        return (self.data[x][y]==-1)
    
    def generateValid(self):
        while True:
            x =randint(0,self.width-1)
            y =randint(0,self.height-1)
            #print "try (%d,%d)" % (x,y)
            if self.is_free(x,y):
                return [x,y]
                
    def generatePair(self):
        while True:
            p1 = self.generateValid()
            p2 = self.generateValid()
            if p1[0]!=p2[0] or p1[1]!=p2[1]:
                return [p1,p2]
        
    def readMap(self,src):
        height = None
        width  = None
        data = None
        for k in range(0,4):
            line = src.readline()
            m = re.match(r"\s*height\s+(\d+)\s*", line)
            if m:
                height = int(m.group(1))
            m = re.match(r"\s*width\s+(\d+)\s*", line)
            if m:
                width = int(m.group(1))
        if width is None or height is None:
            raise Exception("Failed to parse map file: couldn't read width or height")
        else:
            data = [[0 for y in range(height)] for x in range(width)]        
            # width = len(data)
            #height = len(data[0])
                
            for y in range(height-1,-1,-1):
                line = src.readline().rstrip()
                #rospy.loginfo("experimentator: map line (y=%d, length=%d) read= %s", y, len(line), y*width, (y+1)*width-1, str([(-1 if c=="." else -2) for c in line]))
                x=0
                for c in line:
                   data[x][y] = (-1 if c=="." else -10)
                   x+=1 
        #print data
        return data

if __name__=="__main__":
    cmdargs = str(sys.argv)
    mode = sys.argv[1]
    
    gen = RandomPoseGenerator(sys.stdin)
    
    if mode=="pair":
        n = int(sys.argv[2])
        for k in range(n):
            print str(gen.generatePair())
    elif mode=="exp":
        n = int(sys.argv[2])
        exp_prefix = "e"
        map_name = sys.argv[3] #"office2_1.map"
        resolution =   float(sys.argv[4])#0.1
        tries = int(sys.argv[5]) #3
        
        print "experiments: %s" % str(["%s%d"%(exp_prefix,k) for k in range(n)])
        print "experiment_set:"
        for k in range(n):
            print "    %s%d:" % (exp_prefix, k)
            print "        map: \"%s\"" % map_name
            print "        resolution: %f" % resolution
            print "        tries: %d" % tries
            print "        poses: %s" % str(gen.generatePair())
    else:
        sys.stderr.write("Can't read 'mode' (1st argument) from command line")
        