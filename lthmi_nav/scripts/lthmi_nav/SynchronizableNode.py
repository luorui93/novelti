#!/usr/bin/env python
import rospy

from lthmi_nav.srv import *


class SynchronizableNode:
    def __init__(self):
        rospy.loginfo("%s: started a new Experim"% rospy.get_name())
        self.ranBefore = False
        self.start_srv = rospy.Service('~start', StartExperiment, self.startSrv)
    
    def startSrv(self, req):
        if self.ranBefore:
            self.stop()
            rospy.loginfo("%s: stopped Experiment" % rospy.get_name())
            self.ranBefore = True
        self.resolution = req.map.info.resolution
        self.start(req)
        
        rospy.loginfo("%s: started a new Experiment. Init pose=(%f,%f)" % (rospy.get_name(), req.init_pose.position.x, req.init_pose.position.y))
        return StartExperimentResponse()
    
    def start(self, req):
        pass 
    
    def stop(self):
        pass
    
    def run(self):
        rospy.spin()
        
    def pose2vertex(self, pose):
        return ( int(round(pose.position.x / self.resolution)),  int(round(pose.position.y / self.resolution))) 
