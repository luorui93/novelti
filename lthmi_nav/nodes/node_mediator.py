#!/usr/bin/env python

"""


roslaunch anna move_base.launch map:=localization_map init_pose:=[1,1,1]
    amcl -> /pose_current
    
roslaunch lthmi_nav with_move_base.launch map:=inflated_map
    waits for /pose_current
    service request     start
    /pose_desired -> action cancel, action goal

localization_map and inflated_map  should be same  resolution, same origin


ToDo:
    map inflation/synchronization
    move_base_interface
        start experiment
        new goal
        /pose_desired -> action -> /pose_arrived

                         prms: map_file
                              |
    lthmi_nav                 v                   robot
                    +--------------------+
 /pose_desired ---> |                    | <--> action goal
/pose_inferred ---> |                    | 
                    |      mediator      | 
 /pose_current <--- |                    | <--- /amcl_pose
 /pose_arrived <--- |                    | 
                    +--------------------+
                               |
                               v
                           srv: start
                               req:  map
                                   init_pose
                               resp: -
"""

import math
import actionlib
from actionlib_msgs.msg import *
import rospy

import tf_conversions
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from lthmi_nav.SyncingNode import SyncingNode

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Mediator (SyncingNode):
    def __init__(self):
        self.state = "WAIT4START"
        SyncingNode.__init__(self, publishMap=True)
        self.cfg.update({
            'map_file':     rospy.get_param('~map'),
            'resolution':   rospy.get_param('~resolution', 0.1),
            'real_robot':   rospy.get_param('~real_robot', True),
        })
        
        if self.cfg['real_robot']:
            rospy.loginfo("Starting lthmi_nav with a real robot")
            self.action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            self.action_client.wait_for_server()
            
            self.pub_pose_arrived = rospy.Publisher('/pose_arrived', PoseStamped, queue_size=1, latch=True)
            self.pub_pose_current = rospy.Publisher('/pose_current', PoseStamped, queue_size=1, latch=True)
            #self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1, latch=True)
            
            self.sub_pose_desired  = rospy.Subscriber('/pose_desired',  PoseStamped, self.poseDesiredCallback)
            self.sub_pose_inferred = rospy.Subscriber('/pose_inferred', PoseStamped, self.poseInferredCallback)
            self.sub_pose_amcl     = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.poseAmclCallback)
            

        else:
            rospy.loginfo("Starting lthmi_nav without a real robot")
            self.runExperiment(self.cfg['map_file'], self.cfg['resolution'], None)
    
    def startLthmiNav(self, init_pose):
        rospy.loginfo("Starting lthmi_nav")
        self.runExperiment(self.cfg['map_file'], self.cfg['resolution'], init_pose.pose.pose)
        
    def cancelAllGoals(self):
        self.action_client.cancel_all_goals() # don't know why this does not work 
        #g = GoalID()
        #self.pub_cancel.publish(g)
        
    def setNewGoal(self, msg):
        self.setNewGoalInAnotherThread(msg)
        #Thread(target=setNewGoalInAnotherThread, args=[self, msg]).start()
        
    def setNewGoalInAnotherThread(self, msg):
        rospy.loginfo(">>>>>>>>>>> New goal received, canceling all previous goals")
        self.cancelAllGoals()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = msg.pose
        goal.target_pose.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.0))

        rospy.loginfo("Sending destination goal to move_base")
        self.action_client.send_goal(goal, feedback_cb=self.actionFeedbackCallback, done_cb=self.actionDoneCallback)
        #self.action_client.wait_for_result()
        #rospy.logerr(self.action_client.get_state())
        #if self.action_client.get_state() == GoalStatus.SUCCEEDED:
            #rospy.loginfo("Arrived to the destination")
        #else:
            #rospy.loginfo("Failed to arrive to destination")

    def actionDoneCallback(self, p1, p2):
        if self.action_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Arrived to the destination")
        else:
            rospy.loginfo("Failed to arrive to destination")
        
    def poseAmclCallback(self, msg):
        p = PoseStamped()
        p.header = msg.header
        p.pose = msg.pose.pose
        p.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, -math.pi/2, 0.0))
        self.pub_pose_current.publish(p)
        if self.state == "WAIT4START":
            self.state = "RUNNING"
            self.startLthmiNav(msg)
    
    def poseDesiredCallback(self, msg):
        self.setNewGoal(msg)

    def poseInferredCallback(self, msg):
        self.setNewGoal(msg)
    
    def actionFeedbackCallback(self, msg):
        self.latestPose = msg

if __name__=="__main__":
    rospy.init_node('node_mediator')
    e = Mediator()
    rospy.spin()
