#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from low_throughput_hmi.msg import IntMap
from low_throughput_hmi.msg import Experiment
from std_srvs.srv import *

current_pose = None
time_reach = None
optimal_pose = None

def sceneCallback(msg):
    global current_pose, optimal_pose, time_reach, sub_opt_pose, sub_ident_pose
    current_pose = PoseStamped()
    current_pose.header.frame_id="/map"
    current_pose.pose = msg.init_pose
    time_reach = None
    optimal_pose = None
    sub_opt_pose = rospy.Subscriber('/optimal_pose', PoseStamped, optimalPoseCallback)
    sub_ident_pose = rospy.Subscriber('/identified_pose', PoseStamped, identifiedPoseCallback)
    rospy.loginfo("robot_model: got scene, init pose=(%f,%f)", current_pose.pose.position.x, current_pose.pose.position.y)

def optimalPoseCallback(msg):
    global optimal_pose, time_reach
    rospy.loginfo("robot_model: recieved optimal_pose: (%f,%f)" % (msg.pose.position.x, msg.pose.position.y))
    optimal_pose = msg
    time_reach = rospy.get_time() + time_to_dist 

def identifiedPoseCallback(msg):
    global optimal_pose, time_reach
    rospy.loginfo("robot_model: recieved identified_pose: (%f,%f)" % (msg.pose.position.x, msg.pose.position.y))
    optimal_pose = msg
    time_reach = rospy.get_time() + time_to_dist 
    
    
def stop(req):
    global sub_opt_pose, current_pose, sub_ident_pose
    sub_opt_pose.unregister()
    sub_ident_pose.unregister()
    current_pose = None
    time_reach = None
    rospy.loginfo("robot_model: stopped")
    return EmptyResponse()

    
if __name__=="__main__":
    #global pub, time_to_dist, current_pose, time_reach
    
    rospy.init_node('robot_model')
    
    #velocity = rospy.get_param('~velocity', 3.0)
    time_to_dist = rospy.get_param('~timeToDist', 2.0)
    pub_period = rospy.get_param('~pubPeriod', 0.05)
    
    sub_scene = rospy.Subscriber('/scene', Experiment, sceneCallback)
    pub = rospy.Publisher('/current_pose', PoseStamped)
    
    s = rospy.Service('~stop', Empty, stop)
    
    while(not rospy.is_shutdown()):
        if time_reach is not None and rospy.get_time() >= time_reach:
            current_pose = optimal_pose
        if current_pose is not None:
            current_pose.header.stamp = rospy.Time.now()
            pub.publish(current_pose)
            #rospy.loginfo("robot_model: published current_pose: (%f,%f)" % (current_pose.pose.position.x, current_pose.pose.position.y))
        rospy.sleep(pub_period) 

    rospy.spin()
