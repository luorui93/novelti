#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped

optimal_pose = None
track_map = None

def optimalPoseCallback(msg):
    global optimal_pose, time_reach
    rospy.loginfo("robot_model: recieved optimal_pose: (%f,%f)" % (msg.pose.position.x, msg.pose.position.y))
    optimal_pose = msg
    time_reach = rospy.get_time() + time_to_dist 
   
def trackMapCallback(msg):
    pass
   
if __name__=="__main__":
    global pub, time_to_dist, cur_pose, time_reach
    
    rospy.init_node('stochastic_hmi_model')
    
    #velocity = rospy.get_param('~velocity', 3.0)
    time_to_dist = rospy.get_param('~timeToDist', 2.0)
    init_x = rospy.get_param('~initX', 1.0)
    init_y = rospy.get_param('~initY', 1.0)
    cur_pose = PoseStamped()
    cur_pose.header.frame_id="/map"
    cur_pose.pose.position.x = init_x
    cur_pose.pose.position.y = init_y
    pubPeriod = rospy.get_param('~pubPeriod', 0.1)
    
    
    sub1 = rospy.Subscriber('/optimal_pose', PoseStamped, optimalPoseCallback)
    sub2 = rospy.Subscriber('/track_map', IntMap, trackMapCallback)
    pub = rospy.Publisher('/current_pose', PoseStamped)
    
    time_reach = None
    while(not rospy.is_shutdown()):
        if time_reach is not None and rospy.get_time() >= time_reach:
            cur_pose = optimal_pose
        pub.publish(cur_pose)
        #rospy.loginfo("robot_model: published current_pose: (%f,%f)" % (cur_pose.pose.position.x, cur_pose.pose.position.y))
        rospy.sleep(pubPeriod) 
    
    
    rospy.spin()
