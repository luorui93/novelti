#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from lthmi_nav.msg import IntMap
from lthmi_nav.msg import FloatMap

from datetime import datetime
import random
import math

from lthmi_nav.SyncingNode import SyncingNode


class RobotModelTester (SyncingNode):
    def __init__(self):
        SyncingNode.__init__(self, publishMap=True)
        self.cfg.update({
            'map_file':      rospy.get_param('~map'),
            'resolution':    rospy.get_param('~resolution', 0.1),
            'n_experiments': rospy.get_param('~n_experiments', 5),
            'n_poses':       rospy.get_param('~n_poses', 5),
            'delay_min':     rospy.get_param('~delay_min', 0.1),
            'delay_max':     rospy.get_param('~delay_max', 5.0)
        })
        
        self.pub_pose_cur  = rospy.Publisher('/pose_desired', PoseStamped, queue_size=1, latch=True)#, latch=False)
        self.exps_left     = self.cfg['n_experiments']
        self.poses_left    = 0
        random.seed(datetime.now())
        
    
    def publishNext(self):
        if self.poses_left==0:
            if self.exps_left==0:
                exit(0)
            self.runExperiment(self.cfg['map_file'], self.cfg['resolution'], None)
            self.exps_left -= 1
        rospy.loginfo("%s: ============ testing next pose ===========" % (rospy.get_name()))
        self.poses_left -= 1
        self.publishPose()
        rospy.sleep(random.uniform(self.cfg['delay_min'], self.cfg['delay_max']))


    def publishPose(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        vx = self.grid.genRandUnblockedVertex() #[114,22]
        pose.pose = self.vertex2pose(vx)
        pose.header.frame_id="/map"
        self.pub_pose_cur.publish(pose)
        rospy.loginfo("%s: published /pose_desired vertex=(%d,%d), pose=(%f,%f)" % (rospy.get_name(), vx[0], vx[1], pose.pose.position.x, pose.pose.position.y))

        
if __name__=="__main__":
    rospy.init_node('test_robot_model')
    mdt = RobotModelTester()
    while not rospy.is_shutdown():
        mdt.publishNext()
    rospy.spin()
