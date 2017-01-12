#!/usr/bin/env python

import rosbag
import sys
import ast

from math import *

import matplotlib.pyplot as plt
import numpy as np

import tf_conversions

def info(msg):
    sys.stdout.write("INFO: %s\n" % msg)

def warn(msg):
    sys.stderr.write("WARNING: %s\n" % msg)
    
def err(msg):
    sys.stderr.write("ERROR:   %s\n" % msg)

class LthmiNavExpRecord:
    
    def __init__(self, bag_path, **cfg):
        info("Reading bag from %s" % bag_path)
        self.cfg = {
            "path_period": 1.0,
        }
        self.cfg.update(cfg)
        self.bag_path = bag_path
        self.readMetaData(bag_path)
        self.readData(bag_path)
        self.preparePath()
        self.prepareDist()

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
            if "firstCmdIntendedStamp" not in meta and topic=="/cmd_intended":
                meta['firstCmdIntendedStamp'] = msg.header.stamp
            if "firstCmdDetectedStamp" not in meta and topic=="/cmd_detected":
                meta['firstCmdDetectedStamp'] = msg.header.stamp
            if "poseInferredStamp" not in meta and topic=="/pose_inferred":
                meta['poseInferredStamp'] = msg.header.stamp
            if topic=="/pose_intended_goal":
                meta['goal'] = msg.pose
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
            if p>=0:
                e += -p*log(p,2)
        return e
    
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
    
    def prepareDist(self):
        self.dist = { 't':[], 'v':[] }
        for k in xrange(len(self.poses['x'])):
            self.dist['t'].append(self.poses['t'][k])
            self.dist['v'].append(sqrt(  (self.poses['x'][k]-self.meta['goal'].position.x)**2   +   (self.poses['y'][k]-self.meta['goal'].position.y)**2   ))
    
    def preparePath(self):
        dt = self.cfg["path_period"]
        next_t  = dt
        i=0
        #self.path = self.poses
        #return
        self.path = { 't':[0.0], 'x':[self.poses['x'][0]], 'y':[self.poses['y'][0]], 'a':[self.poses['a'][0]] }
        for k,t in enumerate(self.poses['t']):
            #print "===== k=%d,   t=%f,   next_t=%f" % (k,t, next_t)
            #if k==3:
                #break
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
                self.path['t'].append(next_t) 
                self.path['x'].append(x)
                self.path['y'].append(y)
                self.path['a'].append(a)
                next_t += dt
    

    def drawRegularPath(self, axes, color):
        dt = 3.0
        next_t  = dt
        path = self.path
        i=0
        print path['t']
        self.drawArrow(axes, path['x'][0], path['y'][0], path['a'][0], color)
        self.drawArrow(axes, path['x'][-1], path['y'][-1], path['a'][-1], color)
        for k,t in enumerate(path['t']):
            if t >= next_t:
                c    = (next_t - path['t'][k-1])/(t - path['t'][k-1])
                i +=1
                x    = path['x'][k-1] + c*(path['x'][k]-path['x'][k-1])
                y    = path['y'][k-1] + c*(path['y'][k]-path['y'][k-1])
                a0 = path['a'][k-1]
                a1 = path['a'][k]
                #print "%d: a0=%f, a1=%f" %(i, a0, a1)
                if (a0>0 and a1<0) or (a1>0 and a0<0):
                    if a0>0:
                        a0,a1 = a1,a0
                    if a1-a0 > pi:
                        a0 += 2*pi
                a = a0 + c*(a1-a0)
                print "%d: k=%d,  t0=%f, t=%f,      c=%f,     a0=%f, a1=%f,    a=%f" %(i, k, path['t'][k-1], t, c, a0, a1, a)
                self.drawArrow(axes, x,y,a, color)
                #plt.show()
                #time.sleep(1.0)
                next_t += dt


class LthmiNavExpPlot:
    """
    Usage pattern:
        p = LthmiNavExpPlot()
        p.plotBagRecord(bagRecord1)
        p.plotBagRecord(bagRecord2)
        ...
        p.show()
    """
    
    def __init__(self):
        self.colors = [  'blue',  'red', '#5F9ED1', '#ABABAB','#FF800E', '#006B40', 
                    '#FFBC79', '#CFCFCF', '#C85200', '#A2C8EC', '#898989']
        self.styles = ['-', '--', '.-', '-.']
        self.fig = plt.figure(facecolor='white')
        self.ax_map = plt.subplot(121)
        self.ax_dist = plt.subplot(222)
        self.ax_entr = plt.subplot(224, sharex=self.ax_dist)
        self.firstBagRecord = True
        self.bagsDisplayed = 0
    
    def plotBagRecord(self, bagRecord):
        if self.bagsDisplayed == 0:
            self.plotMaps(
                bagRecord.meta['width'], 
                bagRecord.meta['height'], 
                bagRecord.meta['map_inflated'], 
                bagRecord.meta['map']
            )
        self.plotDistance(bagRecord.dist)
        self.plotEntropy(bagRecord.entropy)
        self.plotPath(bagRecord.path)
        self.bagsDisplayed += 1
    
    def plotMaps(self, width, height, map1, map_inflated):
        """
            width and height are sizes of map in meters
            map1 and map_inflated are 2D matrices
        """
        info("    Drawing map")
        self.ax_map.matshow(map_inflated, 
                            origin="lower", 
                            cmap=plt.cm.gray, 
                            vmin=0, vmax=1, norm=None,
                            extent=(0,width,0,height))
        self.ax_map.matshow(map1, 
                            origin="lower", 
                            alpha=0.3, 
                            cmap=plt.cm.gray, 
                            vmin=0, vmax=1, norm=None,
                            extent=(0,width,0,height))
    
    def plotEntropy(self, entropy):
        info("    Drawing entropy plot")
        self.ax_entr.plot(entropy['t'], entropy['v'], 
                          color=self.colors[self.bagsDisplayed], 
                          linestyle=self.styles[0]
                          )
        self.ax_entr.grid(True)
        self.ax_entr.autoscale(True)
        self.ax_entr.set_title("PDF entropy evolution over time", y=1.00)
        self.ax_entr.set_ylabel("Entropy, bits")
        self.ax_entr.set_xlabel("Time, sec")
    
    def plotDistance(self, dist):
        info("    Drawing distance plot")
        self.ax_dist.plot(dist['t'], dist['v'], 
                          color=self.colors[self.bagsDisplayed], 
                          linestyle=self.styles[0]
                          )
        self.ax_dist.grid(True)
        self.ax_dist.autoscale(True)
        self.ax_dist.set_title("Distance to destination over time", y=1.00)
        self.ax_dist.set_ylabel("Distance, m")
    
    def plotPath(self, path):
        for k,t in enumerate(path['t']):
            self.drawArrow(path['x'][k], path['y'][k], path['a'][k], self.colors[self.bagsDisplayed])

    def drawArrow(self, x,y,a, color):
        arr_length  = 0.2
        head_length = 0.07
        head_width  = 0.07
        self.ax_map.arrow(x, y, arr_length*cos(a), arr_length*sin(a), 
                   length_includes_head = True,
                   head_width=head_width, 
                   head_length=head_length, 
                   fc=color, 
                   ec=color)
    
    def show(self):
        plt.tight_layout() 
        plt.show()


doc="""
USAGE:
    PDF distance and entropy plot:
        ./Bag2Plot.py dist_and_entropy 1.bag 2.bag 3.bag ...
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
        else:
            usageError("Incorrect usage")
