#!/usr/bin/env python
import rospy
from std_msgs.msg import String

"""
This is a helper node that allows to save ros parameters in rosbag-files.
It waits for rosbag to start, then it reads all parameters, 
serizlizes them into a string, and publishes as a String message 
to topic /parameters.
Example usage in laucnh file:

    <node pkg="my_pkg" type="node_param_publisher.py" name="node_param_publisher">
        <param name="rosbag_node_name" value="rosbag_record" />
        <!-- any additional values from launch files you want to keep (such as remapped parameters (by default not save on param server) -->
        <param name="prm1" value="$(arg prm1)" />
        <param name="prm2" value="$(arg prm2)" />
    </node>

"""

if __name__=="__main__":
    rospy.init_node('node_param_publisher')
    rosbag_node_name = rospy.get_param('~rosbag_node_name', "/record")
    srv_path = rosbag_node_name+'/get_loggers'
    rospy.loginfo("%s: waiting for service %s (=waiting for rosbag to start)" % (rospy.get_name(), srv_path))
    rospy.wait_for_service(srv_path)
    pub = rospy.Publisher('/parameters', String, queue_size=1, latch=True)
    prms = rospy.get_param('/')
    pub.publish(str(prms))
    rospy.spin()
