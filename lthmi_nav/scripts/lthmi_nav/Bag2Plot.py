#!/usr/bin/env python

import rosbag
import sys

from math import *

import matplotlib.pyplot as plt



def warn(msg):
    sys.stderr.write("WARNING: %s\n" % msg)
    
def err(msg):
    sys.stderr.write("ERROR:   %s\n" % msg)

class BagRecord:
    
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.readMetaData(bag_path)
        self.readData(bag_path)

    def readMetaData(self, bag_path):
        meta = {}
        for topic, msg, t in rosbag.Bag(bag_path).read_messages():
            if "firstPdfStamp" not in meta and topic=="/pdf":
                meta['firstPdfStamp'] = msg.header.stamp
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
        self.entropy = { 't':[], 'v':[] }
        self.dist2goal = { 't':[], 'v':[] }
        for topic, msg, t in rosbag.Bag(bag_path).read_messages():
            if topic=="/pdf"  and  msg.header.stamp>=self.meta['firstPdfStamp']  and  msg.header.stamp<=self.meta['arrivedStamp']:
                t = (msg.header.stamp - self.meta['firstPdfStamp']).to_sec()
                if self.entropy['t']:
                    self.entropy['t'].append(t)
                    self.entropy['v'].append(self.entropy['v'][-1])
                self.entropy['t'].append(t)
                self.entropy['v'].append(self.calcPdfEntropy(msg))
            if topic=="/amcl_pose"  and  msg.header.stamp>=self.meta['firstPdfStamp']  and  msg.header.stamp<=self.meta['arrivedStamp']:
                t = (msg.header.stamp - self.meta['firstPdfStamp']).to_sec()
                self.dist2goal['t'].append(t)
                self.dist2goal['v'].append(self.calcDistToGoal(msg.pose))

    @classmethod
    def makePlots(cls, bag_paths):
        colors = [  'blue', 'blue', 'blue', 'blue', 'yellow', '#ABABAB', '#5F9ED1', '#FF800E', '#006B40', 
                    '#FFBC79', '#CFCFCF', '#C85200', '#A2C8EC', '#898989']
        styles = ['-', '--', '.-', '-.']
        
        fig = plt.figure(facecolor='white')
        ax_map = plt.subplot(121)
        ax_dist = plt.subplot(222)
        ax_entr = plt.subplot(224, sharex=ax_dist)
        
        
        for k, path in enumerate(bag_paths):
            br = cls(path)
            ax_dist.plot(br.dist2goal['t'], br.dist2goal['v'], color=colors[k], linestyle=styles[k])
            ax_dist.grid(True)
            ax_dist.autoscale(True)
            ax_dist.set_title("Distance to destination over time", y=1.00)
            ax_dist.set_ylabel("Distance, m")

            ax_entr.plot(br.entropy['t'], br.entropy['v'], color=colors[k], linestyle=styles[k])
            ax_entr.grid(True)
            ax_entr.autoscale(True)
            ax_entr.set_title("PDF entropy evolution over time", y=1.00)
            ax_entr.set_ylabel("Entropy, bits")
            ax_entr.set_xlabel("Time, sec")
            
        plt.tight_layout() #pad=1.0, h_pad=4.0, w_pad=2.0, rect=(0, 0, 1, 0.95))
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
            BagRecord.makePlots(sys.argv[2:])
        else:
            usageError("Incorrect usage")
