#!/usr/bin/env python
import os
import rospy
import subprocess

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

"""
This is a helper node that allows to save ros parameters in rosbag-files.
It waits for rosbag to start, then it reads all parameters, 
serizlizes them into a string, and publishes as a String message 
to topic /parameters.
"""


if __name__=="__main__":
    global pub
    rospy.init_node('node_param_publisher.py')
    rosbag_node_name = rospy.get_param('~rosbag_node_name', None)
    if rosbag_node_name is None:
        rospy.logerr("parameter rosbag_node_name has to be specified");
        exit(1)
    srv_path =rosbag_node_name+'/get_loggers'
    rospy.loginfo("%s: waiting for service %s" % (rospy.get_name(), srv_path))
    rospy.wait_for_service(srv_path)
    pub = rospy.Publisher('/parameters', String, queue_size=10, latch=True)
    #directory = os.path.dirname(os.path.realpath(__file__))
    #os.chdir(directory)
    #rospy.set_param('git_commit', subprocess.check_output(['git', 'log', '--format="%H"', '-n', '1']))
    #rospy.set_param('git_log', subprocess.check_output(['git', 'log', '-n', '1']))
    prms = rospy.get_param('/')
    pub.publish(str(prms))
    rospy.spin()
