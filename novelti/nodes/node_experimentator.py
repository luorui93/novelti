#!/usr/bin/env python

import math
import rospy

from geometry_msgs.msg import PoseStamped
from novelti.SyncingNode import SyncingNode


class Experimentator (SyncingNode):
    def __init__(self):
        SyncingNode.__init__(self, publishMap=True)
        self.cfg.update({
            'map_file':     rospy.get_param('~map'),
            'resolution':   rospy.get_param('~resolution', 0.1),
            'n_runs':       rospy.get_param('~n_runs', 1),
            'poses':        rospy.get_param('~poses', []),
            #"intended_goal": rospy.get_param('~intended_goal', []),
            'success_file': rospy.get_param('~success_file', None),
        })
        #rospy.loginfo("=======================================%s" % str( self.cfg['poses']))
        self.vxIter = iter([]) #self.cfg['poses'])
        self.goalIter = iter([]) #self.cfg['intended_goal']
        self.state = "INFERRING"
        self.runs_left = self.cfg['n_runs']
        self.pub_pose_intended = rospy.Publisher('/pose_intended_goal', PoseStamped, queue_size=1, latch=True)
        self.pub_position_arrived = rospy.Publisher('/final_position_arrived',PoseStamped,queue_size=1, latch=True)
        self.sub_pose_inferred = rospy.Subscriber('/pose_inferred', PoseStamped, self.poseInferredCallback)
        self.sub_position_inferred = rospy.Subscriber('/position_inferred', PoseStamped, self.positionInferredCallback)
        #self.sub_pose_current  = rospy.Subscriber('/pose_current',  PoseStamped, self.poseCurrentCallback)
        self.sub_pose_arrived  = rospy.Subscriber('/pose_arrived',  PoseStamped, self.poseArrivedCallback)
        self.dst_first_arrival = True
    
    def publishNextIntendedPose(self):
        nextVx = next(self.vxIter, None)
        if nextVx is None:
            if self.runs_left == 0:
                rospy.loginfo("%s: ^^^^^^^^^^^^^^^^^^^ all runs completed ^^^^^^^^^^^^^^^^^^^^^^^^" % (rospy.get_name()))
                if self.cfg['success_file'] is not None:
                    open(self.cfg['success_file'], 'a').close()
                rospy.signal_shutdown("finished")
                return
            self.vxIter = iter(self.cfg['poses'])
            #temporary 
            vx = self.vxIter.next()
            self.runExperiment(self.cfg['map_file'], self.cfg['resolution'], self.vertex2pose(vx))
            self.onExperimentStarted()
            self.runs_left -= 1
            try:
                nextVx = self.vxIter.next()
            except StopIteration:
                rospy.loginfo("Last element in path!")
            else:
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
        if self.state=="POSITION_INFERRED":
            rospy.loginfo("%s: POSITION_INFERRED->POSE_INFERRED detected" % (rospy.get_name()))
            self.pose_inferred = msg.pose
            self.state="INFERRED"
            self.onPoseInferred()
    
    def arePosesSame(self, p1, p2):
        return math.sqrt((p1.position.x-p2.position.x)**2 + (p1.position.y-p2.position.y)**2) < self.cfg['resolution']/20.0
    
    def positionInferredCallback(self, msg):
        if self.state=="INFERRING":
            rospy.loginfo("%s: INFERRING->POSITION_INFERRED detected" % (rospy.get_name()))
            self.position_inferred = msg.pose
            self.state="POSITION_INFERRED"
            self.onPoseInferred()

    # It's not good to use current pose to determine if the robot has arrived the final position because move_base has position tolerance
    # Use information from move_base to determine the time
    # def poseCurrentCallback(self, msg):
    #     if self.state=="POSITION_INFERRED" or self.state=="INFERRED":
    #         if self.arePosesSame(msg.pose, self.position_inferred) and self.dst_first_arrival:
    #             self.publishPositionArrived(msg)
    #             self.dst_first_arrival = False

    def poseArrivedCallback(self, msg):
        rospy.loginfo("%s: received /pose_arrived, state==%s" % (rospy.get_name(), self.state))
        if self.state=="INFERRED":
            rospy.loginfo("%s: reached destination" % (rospy.get_name()))
            self.state="INFERRING"
            self.onInferredReached()
            self.publishNextIntendedPose()

    # def publishPositionArrived(self, msg):
    #     self.pub_position_arrived.publish(msg)

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
