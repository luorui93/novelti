#!/usr/bin/env python

import rosbag
import sys
import ast
import os
import pickle

from math import *
from MapTools import GridMap

import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import StringIO
import tf_conversions
from subprocess import Popen, PIPE, STDOUT
from exceptions import RuntimeError

def info(msg):
    sys.stdout.write("INFO: %s\n" % msg)

def warn(msg):
    sys.stderr.write("WARNING: %s\n" % msg)
    
def err(msg):
    sys.stderr.write("ERROR:   %s\n" % msg)

class LthmiNavExpRecord:
    
    def __init__(self, bag_path, cwave_cmdline_path, **cfg):
        info("Reading bag from %s" % bag_path)
        self.cfg = {
            "path_period": 1.0,
        }
        self.cfg.update(cfg)
        self.cwave_cmdline_path = cwave_cmdline_path
        self.bag_path = bag_path
        self.readMetaData(bag_path)
        self.readData(bag_path)
    
    def getRecord(self):
        return {
            "width"     : self.meta["width"],
            "height"    : self.meta["height"],
            "map"       : self.meta["map"],
            "map_inflated": self.meta["map_inflated"],
            "poses"     : self.poses,
            "path"      : self.calcPath(),
            #"dist"      : self.calcEucDist(),
            "dist"      : self.calcObstDist(),
            "entropy"   : self.entropy,
        }


    def makeMap(self, msg):
        m = np.full((msg.info.height, msg.info.width), 1)
        for x in xrange(msg.info.width):
            for y in xrange(msg.info.height):
                #print msg.data[x + y*msg.info.width]
                if msg.data[x + y*msg.info.width] > 0:
                    m[y,x] = 0
        return m
    
    def makeMapInflated(self, msg):
        m = np.full((msg.info.height, msg.info.width), 0)
        for x in xrange(msg.info.width):
            for y in xrange(msg.info.height):
                if msg.data[x + y*msg.info.width] == 255:
                    m[y,x] = 1.0
        return m

    def readMetaData(self, bag_path):
        info("    Reading meta data")
        meta = {}
        for topic, msg, t in rosbag.Bag(bag_path).read_messages():
            if topic=="/parameters":
                self.prms=ast.literal_eval(msg.data)
                self.initPoseName = self.prms['node_param_publisher']['start_pose_name']
                self.initPose = self.prms['predefined_poses'][self.initPoseName]
                #print self.initPose
            if "firstPdfStamp" not in meta and topic=="/pdf":
                meta['firstPdfStamp'] = msg.header.stamp
                #info("    firstPdfStamp=%f" % msg.header.stamp.to_sec())
            if "map" not in meta and topic=="/map":
                info("        reading map")
                meta['map'] = self.makeMap(msg)
                meta['width'] = msg.info.resolution*msg.info.width
                meta['height'] = msg.info.resolution*msg.info.height
            if "map_inflated" not in meta and topic=="/map_inflated":
                info("        reading map_inflated")
                meta['map_inflated'] = self.makeMapInflated(msg)
                meta['map_inflated_resolution'] = msg.info.resolution
            if "firstCmdIntendedStamp" not in meta and topic=="/cmd_intended":
                meta['firstCmdIntendedStamp'] = msg.header.stamp
            if "firstCmdDetectedStamp" not in meta and topic=="/cmd_detected":
                meta['firstCmdDetectedStamp'] = msg.header.stamp
            if "poseInferredStamp" not in meta and topic=="/pose_inferred":
                meta['poseInferredStamp'] = msg.header.stamp
            if topic=="/pose_intended_goal":
                meta['goal'] = msg.pose
            if topic=="/pose_intended_goal2":
                meta['goal2'] = msg.pose
            if "poseInferredStamp" in meta and topic=="/rosout" and msg.name=="/node_mediator":
                if msg.msg == "Arrived to the destination":
                    pass
                elif msg.msg.startswith("Didn't arrive to destination"):
                    warn("Failed to arrive to the inferred destination: %s" % msg.msg)
                else:
                    continue
                meta['arrivedStamp'] = msg.header.stamp
        self.meta = meta

    def calcPdfEntropy(self, pdf):
        e = 0.0
        for p in pdf.data:
            if p>0: #if p==0,   p*log(p,2) tends to 0
                e += -p*log(p,2)
        return e
    
    def pose2vertex(self, x, y):
        return ( int(round(x / self.meta['map_inflated_resolution'])),  int(round(y / self.meta['map_inflated_resolution']))) 
    
    def calcObstDist(self): 
        #This function is ugly, probably this whole class should be written in C++
        
        #prepare command line parameters to calculate CWave distance
        if "goal2" in self.meta:
            goal = self.meta['goal2']
        else:
            goal = self.meta['goal']
        src = [str(v) for v in self.pose2vertex(goal.position.x, goal.position.y)]
        pts = [i for sub in   [self.pose2vertex(self.poses['x'][k], self.poses['y'][k])  for k in xrange(len(self.poses['x']))]    for i in sub]
        popen_cmd = [self.cwave_cmdline_path] + ["one2many"] + list(src) + [str(val) for val in pts]
        
        #prepare map
        map_h,map_w = self.meta['map_inflated'].shape
        map_data = [GridMap.FREE if cell==1.0 else GridMap.OCCUPIED  for cell in self.meta["map_inflated"].reshape(1, map_w*map_h)[0]]
        gridMap = GridMap(map_w,map_h,map_data)
        mapIoObj = StringIO.StringIO()
        gridMap.printAsText(mapIoObj)
        mapString = mapIoObj.getvalue()
        #print mapString
        
        #run CWave to calculate distances
        p = Popen(popen_cmd, stdout=PIPE, stdin=PIPE, stderr=PIPE)
        #print " ".join(popen_cmd)
        stdout_data = p.communicate(input=mapString)[0]
        #print stdout_data
        if p.returncode != 0:
            #print "Return code of cwave_cmdline tool call is non-zero. It's %d" %p.returncode
            raise RuntimeError("Return code of cwave_cmdline tool call is non-zero. It's %d" %p.returncode)
        
        # form dist array
        dist = { 
            't': self.poses['t'], 
            'v': [self.meta['map_inflated_resolution']*float(s) for s in stdout_data.split()]
        }
        #print dist
        return dist
    
    def calcDistToGoal(self, pose):
        return sqrt(  (pose.pose.position.x-self.meta['goal'].position.x)**2   +   (pose.pose.position.y-self.meta['goal'].position.y)**2   )
    
    def readData(self, bag_path):
        info("    Reading data")
        self.entropy = { 't':[], 'v':[] }
        self.poses = { 't':[0.0], 'x':[self.initPose['x']], 'y':[self.initPose['y']], 'a':[self.initPose['yaw']] }
        for topic, msg, t in rosbag.Bag(bag_path).read_messages():
            if topic=="/pdf"  and  msg.header.stamp>=self.meta['firstPdfStamp']  and  msg.header.stamp<=self.meta['arrivedStamp']:
                t = (msg.header.stamp - self.meta['firstPdfStamp']).to_sec()
                if self.entropy['t']:
                    self.entropy['t'].append(t)
                    self.entropy['v'].append(self.entropy['v'][-1])
                self.entropy['t'].append(t)
                self.entropy['v'].append(self.calcPdfEntropy(msg))
            if topic=="/amcl_pose" and msg.header.stamp>=self.meta['firstPdfStamp']  and  msg.header.stamp<=self.meta['arrivedStamp']:
                #print "first pose in path has time stamp = : %f" % msg.header.stamp.to_sec()
                t = (msg.header.stamp - self.meta['firstPdfStamp']).to_sec()
                self.poses['t'].append(t)
                self.poses['x'].append(msg.pose.pose.position.x)
                self.poses['y'].append(msg.pose.pose.position.y)
                (r, p, y) = tf_conversions.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                self.poses['a'].append(y)
    
    def calcEucDist(self):
        dist = { 't':[], 'v':[] }
        for k in xrange(len(self.poses['x'])):
            dist['t'].append(self.poses['t'][k])
            dist['v'].append(sqrt(  (self.poses['x'][k]-self.meta['goal'].position.x)**2   +   (self.poses['y'][k]-self.meta['goal'].position.y)**2   ))
        return dist
    
    def calcPath(self):
        dt = self.cfg["path_period"]
        next_t  = dt
        i=0

        path = { 't':[0.0], 'x':[self.poses['x'][0]], 'y':[self.poses['y'][0]], 'a':[self.poses['a'][0]] }
        for k,t in enumerate(self.poses['t']):
            #print "===== k=%d,   t=%f,   next_t=%f" % (k,t, next_t)
            while t >= next_t:
                c    = (next_t - self.poses['t'][k-1])/(t - self.poses['t'][k-1])
                i += 1
                x    = self.poses['x'][k-1] + c*(self.poses['x'][k]-self.poses['x'][k-1])
                y    = self.poses['y'][k-1] + c*(self.poses['y'][k]-self.poses['y'][k-1])
                #print "k=%d,    c=%f     y[k-1]=%f, y[k]=%f, " % (k, c, self.poses['y'][k-1], self.poses['y'][k])
                a0 = self.poses['a'][k-1]
                a1 = self.poses['a'][k]
                #print "%d: a0=%f, a1=%f" %(i, a0, a1)
                if (a0>0 and a1<0) or (a1>0 and a0<0):
                    if a0>0:
                        a0,a1 = a1,a0
                    if a1-a0 > pi:
                        a0 += 2*pi
                a = a0 + c*(a1-a0)
                #print "%d: k=%d,  t0=%f, t=%f,      c=%f,     a0=%f, a1=%f,    a=%f" %(i, k, self.poses['t'][k-1], t, c, a0, a1, a)
                path['t'].append(next_t) 
                path['x'].append(x)
                path['y'].append(y)
                path['a'].append(a)
                next_t += dt
        return path


#class LthmiNavExpPlot:
    #"""
    #Usage pattern:
        #p = LthmiNavExpPlot()
        #p.plotBagRecord(bagRecord1, color1)
        #p.plotBagRecord(bagRecord2, color2)
        #...
        #p.show()
    #"""
    
    #def __init__(self):
        ##self.baseDir = baseDir
        ##if baseDir is not None:
            ##self.readListOfAllBagFiles()
        
        #self.colors = [  'blue',  'red', '#5F9ED1', '#ABABAB','#FF800E', '#006B40', 
                    #'#FFBC79', '#CFCFCF', '#C85200', '#A2C8EC', '#898989']
        #self.styles = ['-', '--', '.-', '-.']
        #self.fig = plt.figure(facecolor='white')
        #self.ax_map = plt.subplot(121)
        #self.ax_dist = plt.subplot(222)
        #self.ax_entr = plt.subplot(224, sharex=self.ax_dist)
        #self.firstBagRecord = True
        #self.bagsDisplayed = 0

    
    #def plotBagRecord(self, bagRecord, color):
        #if self.bagsDisplayed == 0:
            #self.plotMaps(
                #bagRecord['width'], 
                #bagRecord['height'], 
                #bagRecord['map_inflated'], 
                #bagRecord['map']
            #)
        #self.plotDistance(bagRecord["dist"], color)
        #self.plotEntropy(bagRecord["entropy"], color)
        #self.plotPath(bagRecord["path"], color)
        #self.bagsDisplayed += 1
    
    #def plotMaps(self, width, height, map1, map_inflated):
        #"""
            #width and height are sizes of map in meters
            #map1 and map_inflated are 2D matrices
        #"""
        #info("    Drawing map")
        #self.ax_map.matshow(map_inflated, 
                            #origin="lower", 
                            #cmap=plt.cm.gray, 
                            #vmin=0, vmax=1, norm=None,
                            #extent=(0,width,0,height))
        #self.ax_map.matshow(map1, 
                            #origin="lower", 
                            #alpha=0.3, 
                            #cmap=plt.cm.gray, 
                            #vmin=0, vmax=1, norm=None,
                            #extent=(0,width,0,height))
    
    #def plotEntropy(self, entropy, color):
        #info("    Drawing entropy plot")
        #self.ax_entr.plot(entropy['t'], entropy['v'], 
                          #color=color, 
                          #linestyle=self.styles[0]
                          #)
        #self.ax_entr.grid(True)
        #self.ax_entr.autoscale(True)
        #self.ax_entr.set_title("PDF entropy evolution over time", y=1.00)
        #self.ax_entr.set_ylabel("Entropy, bits")
        #self.ax_entr.set_xlabel("Time, sec")
    
    #def plotDistance(self, dist, color):
        #info("    Drawing distance plot")
        #self.ax_dist.plot(dist['t'], dist['v'], 
                          #color=color, 
                          #linestyle=self.styles[0]
                          #)
        #self.ax_dist.grid(True)
        #self.ax_dist.autoscale(True)
        #self.ax_dist.set_title("Distance to destination over time", y=1.00)
        #self.ax_dist.set_ylabel("Distance, m")
    
    #def plotPath(self, path, color):
        #for k,t in enumerate(path['t']):
            #self.drawArrow(path['x'][k], path['y'][k], path['a'][k], color)

    #def drawArrow(self, x,y,a, color):
        #arr_length  = 0.2
        #head_length = 0.07
        #head_width  = 0.07
        #self.ax_map.arrow(x, y, arr_length*cos(a), arr_length*sin(a), 
                   #length_includes_head = True,
                   #head_width=head_width, 
                   #head_length=head_length,
                   #linewidth=0.001,
                   #joinstyle='miter', #['miter' | 'round' | 'bevel']
                   #capstyle='butt', #['butt' | 'round' | 'projecting']   http://stackoverflow.com/a/10297860/5787022
                   #fc=color, 
                   #ec=color)
    
    #def show(self):
        #plt.tight_layout() 
        #plt.show()



class RecordByStamp:
    
    def __init__(self, bagDir, cacheDir, cwaveToolPath):
        self.bagDir = bagDir
        self.cacheDir = cacheDir
        self.cwaveToolPath = cwaveToolPath
        self.time2file = []
        for filename in os.listdir(self.bagDir):
            if filename.endswith(".bag") and os.path.isfile(os.path.join(self.bagDir, filename)):
                t = datetime.strptime(filename, '%Y-%m-%d-%H-%M-%S.bag')
                self.time2file.append((t,filename))
        self.time2file.sort(key=lambda tup: tup[0]) 

    def getRecord(self, timeStr):
        t = datetime.strptime(timeStr, '%Y-%m-%d_%H-%M-%S_EST-0500')
        for tup in self.time2file:
            if tup[0]>=t:
                bagFileName = tup[1]
                bagFilePath = os.path.join(self.bagDir, bagFileName)
                pklFileName = bagFileName+".pkl"
                pklFilePath = os.path.join(self.cacheDir, pklFileName)
                if os.path.isfile(pklFilePath):
                    info("Reading data from cache file %s" % pklFilePath)
                    with open(pklFilePath, 'rb') as input1:
                        return pickle.load(input1) 
                else:
                    b = LthmiNavExpRecord(bagFilePath, self.cwaveToolPath)
                    bagRecord = b.getRecord()
                    info("Saving data to cache as %s" % pklFilePath)
                    with open(pklFilePath, 'wb') as output:
                        pickle.dump(bagRecord, output, pickle.HIGHEST_PROTOCOL)
                    return bagRecord
        return None
    

class PlottableBagRecord:
    
    def __init__(self, bagRecord, color=None):
        self.bagRecord = bagRecord
        self.color = color

    def plotMap(self, axes):
        axes.matshow(self.bagRecord['map'], 
                            origin="lower", 
                            cmap=plt.cm.gray, 
                            vmin=0, vmax=1, norm=None,
                            extent=(0, self.bagRecord['width'], 0, self.bagRecord['height']))
            
    def plotMapInflated(self, axes):
        axes.matshow(self.bagRecord['map_inflated'], 
                            origin="lower", 
                            alpha=0.3,
                            cmap=plt.cm.gray, 
                            vmin=0, vmax=1, norm=None,
                            extent=(0, self.bagRecord['width'], 0, self.bagRecord['height']))
        
    def plotMaps(self, axes):
        #info("    Drawing maps")
        self.plotMap(axes)
        self.plotMapInflated(axes)
        
        

    def drawArrow(self, axes, x,y,a, color):
        arr_length  = 0.16
        head_length = 0.08
        head_width  = 0.08
        axes.arrow(x, y, arr_length*cos(a), arr_length*sin(a), 
                   length_includes_head = True,
                   head_width=head_width, 
                   head_length=head_length, 
                   linewidth=0.1,
                   #joinstyle='miter', #['miter' | 'round' | 'bevel']
                   #capstyle='butt', #['butt' | 'round' | 'projecting']   http://stackoverflow.com/a/10297860/5787022
                   fc=color, 
                   ec=color)

    def plotEntropy(self, axes, color):
        #info("    Drawing entropy plot")
        l = axes.plot(self.bagRecord['entropy']['t'], self.bagRecord['entropy']['v'], 
                          color=color, 
                          linestyle='-'
                          )
        axes.grid(True)
        axes.autoscale(True)
        #axes.set_title("PDF entropy evolution over time", y=1.00)
        axes.set_ylabel("PDF entropy, bits", fontsize=16)
        axes.set_xlabel("Time, sec", fontsize=16)
        return l
    
    def plotDistance(self, axes, color):
        #info("    Drawing distance plot")
        l = axes.plot(self.bagRecord['dist']['t'], self.bagRecord['dist']['v'], 
                          color=color, 
                          linestyle='-'
                 )
        axes.grid(True)
        axes.autoscale(True)
        #axes.set_title("Distance to destination over time", y=1.00)
        axes.set_ylabel("Distance to goal, m", fontsize=16)
        axes.set_xlabel("Time, sec", fontsize=16)
        return l
    
    def plotPath(self, axes, color):
        for k,t in enumerate(self.bagRecord['path']['t']):
            self.drawArrow(axes, self.bagRecord['path']['x'][k], self.bagRecord['path']['y'][k], self.bagRecord['path']['a'][k], color)

    def plotPathDistEntropy(self, ax_map, ax_dist, ax_entr, color):
        self.plotPath(ax_map, color)
        line_dist = self.plotDistance(ax_dist, color)
        line_entr = self.plotEntropy(ax_entr, color)
        return (line_dist, line_entr)
    
    
    

class GroupPlot:
    
    def __init__(self, bagDir, cacheDir, cwaveToolPath):
        self.dbase = RecordByStamp(bagDir, cacheDir, cwaveToolPath)
        self.plot  = LthmiNavExpPlot()
        
    def plotArray(self, timeStrs, color):
        for timeStr in timeStrs:
            bagRecord = self.dbase.getRecord(timeStr)
            self.plot.plotBagRecord(bagRecord, color)
    
    def show(self):
        self.plot.show()

doc="""
USAGE:
    PDF distance and entropy plot:
        ./Bag2Plot.py dist_and_entropy 1.bag 2.bag 3.bag ...
        ./Bag2Plot.py from_stamps  <PATH to DIR WITH BAGS>  2017-01-01_16-13-27_EST-0500=black 
"""


def usageError(msg):
    err("%s\nSee usage info below." % msg)
    sys.stdout.write(doc)
    exit(1)


if __name__=="__main__":
    nargs = len(sys.argv)-1
    if len(sys.argv)<2:
        sys.stderr.write(doc)
        exit(1)
    else:
        action = sys.argv[1]
        if action=="help":
            sys.stdout.write(doc)
        elif action=="dist_and_entropy":
            if len(sys.argv)<3:
                usageError("Provide at least one path to a bag file")
            info("Reading bags")
            bagRecords = [LthmiNavExpRecord(filepath) for filepath in sys.argv[2:]]
            info("Making plots")
            p = LthmiNavExpPlot()
            for br in bagRecords:
                p.plotBagRecord(br)
            p.show()
        elif action=="make_pickles":
            if len(sys.argv)<3:
                usageError("Provide at least one path to a bag file")
            info("Reading bags")
            for bagpath in sys.argv[2:]:
                br = LthmiNavExpRecord(bagpath)
                info("Saving data to %s" % bagpath+'.pkl')
                with open(bagpath+'.pkl', 'wb') as output:
                    pickle.dump(br, output, pickle.HIGHEST_PROTOCOL)
        else:
            usageError("Incorrect usage")
