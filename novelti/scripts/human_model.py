#!/usr/bin/env python

import rospy, tf

from novelti.msg import Command
from geometry_msgs.msg import PoseStamped
from novelti.msg import IntMap
from novelti.msg import Experiment
from std_srvs.srv import *
import math



def mayBeAct():
    global INTENDED_POSE, DIVIDED_MAP, NUM_DECISIONS, PUB
    if (INTENDED_POSE is not None and DIVIDED_MAP is not None):
        msg2pub = Command()
        xy = pose2xy(INTENDED_POSE, DIVIDED_MAP.info.resolution)
        msg2pub.cmd = DIVIDED_MAP.data[ xy[0] + DIVIDED_MAP.info.width*xy[1] ]
        PUB.publish(msg2pub)
        NUM_DECISIONS += 1
        rospy.loginfo("human_model: published intended_command = %d, it's my %d'th decision for this intended destination", msg2pub.cmd, NUM_DECISIONS)
        #DIVIDED_MAP = None

def intendedPoseCallback(msg):
    global INTENDED_POSE, NUM_DECISIONS
    NUM_DECISIONS = 0
    INTENDED_POSE = msg
    rospy.loginfo("human_model: recevied new intended_pose =(%f,%f)" % (msg.pose.position.x, msg.pose.position.y))
    #mayBeAct()

def pose2xy(pose, resolution):
    return (
        int(math.floor(pose.pose.position.x/resolution)),
        int(math.floor(pose.pose.position.y/resolution))
    )
    
def dividedMapCallback(msg):
    global intended_pos, num_decisions, DIVIDED_MAP
    DIVIDED_MAP = msg
    rospy.loginfo("human_model: recevied divided_map")
    mayBeAct()
    
def sceneCallback(msg):
    global INTENDED_POSE, DIVIDED_MAP, NUM_DECISIONS, SUB_MAP, SUB_POSE
    rospy.loginfo("human_model: got scene, restarting")
    INTENDED_POSE = None
    NUM_DECISIONS = 0
    DIVIDED_MAP = None
    SUB_MAP = rospy.Subscriber('/divided_map', IntMap, dividedMapCallback)
    SUB_POSE = rospy.Subscriber('/intended_pose', PoseStamped, intendedPoseCallback)

def stop(req):
    global NUM_DECISIONS, SUB_MAP, SUB_POSE
    NUM_DECISIONS = 0
    SUB_MAP.unregister()
    SUB_POSE.unregister()
    rospy.loginfo("human_model: stopped")
    return EmptyResponse()


if __name__=="__main__":
    global PUB
    rospy.init_node('human_model')
    sub1    = rospy.Subscriber('/scene', Experiment, sceneCallback)
    PUB = rospy.Publisher('/intended_command', Command)
    s = rospy.Service('~stop', Empty, stop)
    rospy.spin()
