#!/usr/bin/env python

import math
import rospy

from geometry_msgs.msg import PoseStamped
from lthmi_nav.SyncingNode import SyncingNode


class Experimentator (SyncingNode):
    def __init__(self):
        SyncingNode.__init__(self, publishMap=True)
        self.cfg.update({
            'map_file':     rospy.get_param('~map'),
            'resolution':   rospy.get_param('~resolution', 0.1),
            'n_runs':       rospy.get_param('~n_runs', 1),
            'poses':        rospy.get_param('~poses', []),
        })
        #rospy.loginfo("=======================================%s" % str( self.cfg['poses']))
        self.vxIter = iter([]) #self.cfg['poses'])
        self.state = "INFERRING"
        self.runs_left = self.cfg['n_runs']
        self.pub_pose_intended = rospy.Publisher('/pose_intended', PoseStamped, queue_size=1, latch=False)
        self.sub_pose_inferred = rospy.Subscriber('/pose_inferred', PoseStamped, self.poseInferredCallback)
        self.sub_pose_current  = rospy.Subscriber('/pose_current',  PoseStamped, self.poseCurrentCallback)
    
    def publishNextIntendedPose(self):
        nextVx = next(self.vxIter, None)
        if nextVx is None:
            if self.runs_left == 0:
                exit(0)
            self.vxIter = iter(self.cfg['poses'])
            vx = self.vxIter.next()
            self.runExperiment(self.cfg['map_file'], self.cfg['resolution'], self.vertex2pose(vx))
            self.onExperimentStarted()
            self.runs_left -= 1
            nextVx = self.vxIter.next()
        self.publishPose(nextVx)

    def publishPose(self, vx):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id="/map"
        pose.pose = self.vertex2pose(vx)
        self.pub_pose_intended.publish(pose)
        self.onPoseIntended()
        rospy.loginfo("%s:  ============ published /pose_intended vertex=(%d,%d), pose=(%f,%f)  ============ " % (rospy.get_name(), vx[0], vx[1], pose.pose.position.x, pose.pose.position.y))

    def poseInferredCallback(self, msg):
        if self.state=="INFERRING":
            self.pose_inferred = msg.pose
            self.state="INFERRED"
            self.onPoseInferred()
    
    def arePosesSame(self, p1, p2):
        return sqrt((p1.position.x-p2.position.x)**2 + (p1.position.y-p2.position.y)**2) < self.resolution/20.0
    
    def poseCurrentCallback(self, msg):
        if self.state=="INFERRED" and arePosesSame(msg.pose, self.pose_inferred):
            self.state="INFERRING"
            self.onInferredReached()
            self.publishNextIntendedPose()


    def onExperimentStarted(self):
        pass
    
    def onPoseIntended(self):
        pass
    
    def onPoseInferred(self):
        pass

    def onInferredReached(self):
        pass

if __name__=="__main__":
    rospy.init_node('experimentator')
    e = Experimentator()
    e.publishNextIntendedPose()
    rospy.spin()
