#!/usr/bin/env python

import rospy, tf

import random

from low_throughput_hmi.msg import FloatMap
from geometry_msgs.msg import PoseStamped
from low_throughput_hmi.msg import IntMap

intended_pose = None
intended_point = None
cur_prob_at_dest = None

def publishIntendedDestination():
    global intended_pose, intended_point
    while True:
        x = int(the_map.info.width * random.random())
        y = int(the_map.info.height * random.random())
        if  the_map.data[the_map.info.width*y + x] >=-1:
            p = PoseStamped()
            p.header.frame_id="/map"
            p.pose.position.x = the_map.info.resolution*x
            p.pose.position.y = the_map.info.resolution*y
            intended_pose = p
            intended_point=[x,y]
            pub.publish(p)
            break

def pdfCallback(msg):
    global cur_prob_at_dest
    if intended_pose is not None:
        cur_prob_at_dest =msg.data[msg.info.width*intended_point[1] + intended_point[0]]
   
def curPoseCallback(cur_pose):
    if intended_pose is not None and \
            cur_pose.pose.position.x == intended_pose.pose.position.x and \
            cur_pose.pose.position.y == intended_pose.pose.position.y and \
            cur_prob_at_dest is not None and \
            cur_prob_at_dest>= prob_threshold:
        publishIntendedDestination()
        
        
        
def mapCallback(msg):
    global the_map
    the_map = msg
    if intended_pose is None:
        publishIntendedDestination()
    
if __name__=="__main__":
    global pub, prob_threshold
    
    prob_threshold  = rospy.get_param('~prob_threshold', 0.95)
    
    sub1 = rospy.Subscriber('/map', IntMap, mapCallback)
    sub2 = rospy.Subscriber('/pdf', FloatMap, pdfCallback)
    sub3 = rospy.Subscriber('/current_pose', PoseStamped, curPoseCallback)
    pub = rospy.Publisher('/intended_pose', PoseStamped)
    rospy.init_node('destination_generator')
    
    rospy.spin()
