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
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from lthmi_nav.SyncingNode import SyncingNode


class Mediator (SyncingNode):
    def __init__(self):
        self.state = "WAIT4START"
        SyncingNode.__init__(self, publishMap=True)
        self.cfg.update({
            'map_file':     rospy.get_param('~map'),
            'resolution':   rospy.get_param('~resolution', 0.1),
        })
        self.action_client = actionlib.SimpleActionClient('/move_base', move_base_msgs/MoveBaseGoal)
        self.action_client.wait_for_server()
        
        self.sub_pose_desired  = rospy.Subscriber('/pose_desired',  PoseStamped, self.poseDesiredCallback)
        self.sub_pose_inferred = rospy.Subscriber('/pose_inferred', PoseStamped, self.poseInferredCallback)
        self.sub_pose_amcl     = rospy.Subscriber('/pose_amcl', PoseStamped, self.poseAmclCallback)
        
        self.pub_pose_arrived = rospy.Publisher('/pose_arrived', PoseStamped, queue_size=1, latch=True)
        #self.sub_pose_current  = rospy.Subscriber('/pose_current',  PoseStamped, self.poseCurrentCallback)
    
    def startLthmiNav(self, init_pose):
        self.runExperiment(self.cfg['map_file'], self.cfg['resolution'], init_pose.pose)
        
    def setNewGoal(self, msg):
        self.action_client.cancel_all_goals()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = msg.pose

        rospy.loginfo("Sending destination goal to move_base")
        self.action_client.sendGoal(goal, feedback_cb=self.actionFeedbackCallback)
        self.action_client.waitForResult()

        if self.action_client.getState() == SUCCEEDED:
            rospy.loginfo("Arrived to the destination")
        else:
            rospy.loginfo("Failed to arrive to destination")

    def poseCurrentCallback(self, msg):
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
