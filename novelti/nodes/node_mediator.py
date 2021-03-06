#!/usr/bin/env python

"""


roslaunch anna move_base.launch map:=localization_map init_pose:=[1,1,1]
    amcl -> /pose_current
    
roslaunch novelti with_move_base.launch map:=inflated_map
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
    novelti                 v                   robot
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
import dynamic_reconfigure.client

import tf_conversions
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
from novelti.SyncingNode import SyncingNode

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

class Mediator (SyncingNode):
    GOAL_STATUSES = ["PENDING         = 0   # The goal has yet to be processed by the action server",
                     "ACTIVE          = 1   # The goal is currently being processed by the action server",
                     "PREEMPTED       = 2   # The goal received a cancel request after it started executing and has since completed its execution (Terminal State)",
                     "SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)",
                     "ABORTED         = 4   # The goal was aborted during exe   cution by the action server due to some failure (Terminal State)",
                    "REJECTED        = 5   # The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)",
                    "PREEMPTING      = 6   # The goal received a cancel request after it started executing and has not yet completed execution",
                    "RECALLING       = 7   # The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled",
                    "RECALLED        = 8   # The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)",
                    "LOST            = 9   # An action client can determine that a goal is LOST. This should not be sent over the wire by an action server"]
    
    def __init__(self):
        self.state = "WAIT4START"
        SyncingNode.__init__(self, publishMap=True)
        self.cfg.update({
            'map_file':     rospy.get_param('~map'),
            'resolution':   rospy.get_param('~resolution', 0.1),
            'real_robot':   rospy.get_param('~real_robot', True),
            'pub_random_goal':   rospy.get_param('~pub_random_goal', False),
        })
        
        self.inferenceState="INFERRING_POSITION"
        if self.cfg['real_robot']:
            rospy.loginfo("Starting novelti with a real robot")
            self.action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            self.action_client.wait_for_server()
            
            self.pub_pose_arrived = rospy.Publisher('/pose_arrived', PoseStamped, queue_size=1, latch=True)
            self.pub_pose_current = rospy.Publisher('/pose_current', PoseStamped, queue_size=1, latch=True)
            #self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1, latch=True)
            
            self.sub_position_desired = rospy.Subscriber('/position_desired', PoseStamped, self.positionDesiredCallback)
            self.sub_pose_desired  = rospy.Subscriber('/pose_desired',  PoseStamped, self.poseDesiredCallback)
            self.sub_pose_inferred = rospy.Subscriber('/pose_inferred', PoseStamped, self.poseInferredCallback)
            self.sub_pose_amcl     = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.poseAmclCallback)
        else:
            rospy.loginfo("Starting novelti without a real robot")
            self.runExperiment(self.cfg['map_file'], self.cfg['resolution'], None)
        
    
    def publishRandomGoal(self):
        init_vx = self.grid.genRandUnblockedVertex()
        init_pose = self.vertex2pose(init_vx)
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "/map"
        p.pose = init_pose
        self.pub_pose_goal.publish(p)
    
    def startNovelti(self, init_pose):
        rospy.loginfo("Starting novelti")
        self.runExperiment(self.cfg['map_file'], self.cfg['resolution'], init_pose.pose.pose)
        
    def cancelAllGoals(self):
        self.action_client.cancel_goals_at_and_before_time(rospy.Time.now()) # don't know why this does not work 
        #g = GoalID()
        #self.pub_cancel.publish(g)
        
    def setNewGoal(self, msg):
        self.setNewGoalInAnotherThread(msg)
        #Thread(target=setNewGoalInAnotherThread, args=[self, msg]).start()
        
    def setNewGoalInAnotherThread(self, msg):
        rospy.loginfo("New goal received, canceling all previous goals")
        #self.cancelAllGoals()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = msg.pose
        #goal.target_pose.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.0))

        rospy.loginfo("Sending destination goal to move_base")
        self.action_client.send_goal(goal, feedback_cb=self.actionFeedbackCallback, done_cb=self.actionDoneCallback)
        #self.action_client.wait_for_result()
        #rospy.logerr(self.action_client.get_state())
        #if self.action_client.get_state() == GoalStatus.SUCCEEDED:
            #rospy.loginfo("Arrived to the destination")
        #else:
            #rospy.loginfo("Failed to arrive to destination")

    def actionDoneCallback(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Arrived to the destination")
        else:
            rospy.loginfo("Didn't arrive to destination, goal status=%d: '%s'" % (status, self.GOAL_STATUSES[status]))
        if self.inferenceState=="POSE_INFERRED" and self.cfg['pub_random_goal']:
            rospy.logwarn("PUBLISHING A NEW RANDOM GOAL")
            self.publishRandomGoal()
            self.inferenceState="INFERRING_POSITION"
        
    def poseAmclCallback(self, msg):
        p = PoseStamped()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = msg.header.frame_id
        p.pose = msg.pose.pose
        #p.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.0))
        self.pub_pose_current.publish(p)
        if self.state == "WAIT4START":
            self.state = "RUNNING"
            self.startNovelti(msg)
            #start service for emotiv
            self.readyForEmotiv = rospy.Service("start_emotiv_srv", Empty, self.empty_handler)

    def positionDesiredCallback(self, msg):
        rospy.loginfo("Received position desired: (%f,%f)" % (msg.pose.position.x, msg.pose.position.y))
        self.setNewGoal(msg)

    def poseDesiredCallback(self, msg):
        if self.inferenceState == "INFERRING_POSITION":
            self.inferenceState = "INFERRING_POSE"
            #dynamic reconfigure the yaw_goal_tolerance
            client = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
            params = { 'yaw_goal_tolerance' : '0.01' }
            config = client.update_configuration(params)

        (roll,pitch,yaw) = euler_from_quaternion(
            [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
        rospy.loginfo("Received pose desired: (%f,%f,%f)" % (msg.pose.position.x, msg.pose.position.y, yaw))
        self.setNewGoal(msg)

    def poseInferredCallback(self, msg):
        rospy.loginfo("Received pose inferred: (%f,%f)" % (msg.pose.position.x, msg.pose.position.y))
        self.inferenceState="POSE_INFERRED"
        self.setNewGoal(msg)
    
    def actionFeedbackCallback(self, msg):
        self.latestPose = msg

    def empty_handler(self, req):
        return True
if __name__=="__main__":
    rospy.init_node('node_mediator')
    e = Mediator()
    rospy.spin()
