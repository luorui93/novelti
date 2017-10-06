#!/usr/bin/env python

import re
import math
from threading import Thread
import csv


import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import *
from novelti.msg import IntMap
from novelti.msg import Experiment


class Monitor:
    def __init__(self):
        rospy.init_node('monitor')
        rospy.on_shutdown(self.on_shutdown)
        self.output_prefix  = rospy.get_param('~output_prefix', '/tmp/experiment_results_')
        
        sub_scene         = rospy.Subscriber('/scene',              Experiment,     self.sceneCb)
        
        s = rospy.Service('~stop', Empty, self.stop)
        self.exp_name = None
        self.timing_writer_fh = None
        self.state = None
        self.resolution = None
        self.sub_intended_pose = None
        self.sub_ident_pose    = None
        self.sub_current_pose  = None
        self.timing_totals = None
        self.attempts = 0
        
        fname = self.output_prefix + '_AVERAGES.csv'
        self.timing_avg_writer_fh = open(fname, 'wb')
        self.timing_avg_writer = csv.writer(self.timing_avg_writer_fh)
        
        rospy.spin()
    
    def add_averages(self):
        for k in range(len(self.timing_totals)):
            self.timing_totals[k] = self.timing_totals[k]/self.attempts
        self.timing_avg_writer.writerow(self.timing_totals)
        rospy.logwarn("monitor: --------------------- wrote a line of averages: %s" % str(self.timing_totals))
    
    def on_shutdown(self):
        self.addline()
        self.add_averages()
        #self.timing_avg_writer_fh.close()

    
    def addline(self):
        self.timing_writer.writerow(self.timing_writer_row)
        if self.attempts==0:
            self.timing_totals = self.timing_writer_row
        else:
            for k in range(len(self.timing_writer_row)):
                self.timing_totals[k] += self.timing_writer_row[k]
        self.attempts += 1
        #rospy.logwarn("monitor: -----------")
        
    def sceneCb(self, msg):
        self.state = "WAIT_FOR_START"
        self.resolution = msg.mapa.info.resolution
        
        if self.exp_name != msg.name:
            self.exp_name = msg.name
            if self.timing_writer_fh is not None:
                self.addline()
                self.timing_writer_fh.close()
            fname = self.output_prefix + self.exp_name + '_timings.csv'
            self.timing_writer_fh = open(fname, 'wb')
            self.timing_writer = csv.writer(self.timing_writer_fh)
            rospy.loginfo("monitor: starting a new file at %s", fname)
            
            if self.timing_totals is not None:
                self.add_averages()
            self.attempts = 0
        else:
            self.addline()
        self.timing_writer_row = []
        
        
        self.sub_intended_pose = rospy.Subscriber('/intended_pose',      PoseStamped,    self.intendedPoseCb)
        self.sub_ident_pose    = rospy.Subscriber('/identified_pose',    PoseStamped,    self.identifiedPoseCb)
        self.sub_current_pose  = rospy.Subscriber('/current_pose',       PoseStamped,    self.currentPoseCb)

    def intendedPoseCb(self, msg):
        self.state = "WAIT_FOR_IDENTIFIED_POSE"
        self.start_time = msg.header.stamp
        self.next_pose = self.pose2xy(msg)
        
    def identifiedPoseCb(self, msg):
        self.state = "WAIT_FOR_ARRIVAL"

    def currentPoseCb(self, msg):
        if self.state == "WAIT_FOR_ARRIVAL":
            xy = self.pose2xy(msg)
            if xy[0]==self.next_pose[0] and xy[1]==self.next_pose[1]:
                self.state = "ARRIVED"
                #rospy.logwarn("monitor: ----------- append")
                self.timing_writer_row.append((msg.header.stamp - self.start_time).to_sec())
        
    def xy_to_pose(self, xy):
        p = Pose()
        p.position.x = (xy[0]+0.5)*self.resolution
        p.position.y = (xy[1]+0.5)*self.resolution
        return p

    def pose2xy(self, pose):
        return (
            int(math.floor(pose.pose.position.x/self.resolution)),
            int(math.floor(pose.pose.position.y/self.resolution))
        )

    def stop(self, req):
        self.sub_intended_pose.unregister()
        self.sub_ident_pose.unregister()
        self.sub_current_pose.unregister()
        rospy.loginfo("Monitor: stopped")
        return EmptyResponse()


if __name__=="__main__":
    m = Monitor()
