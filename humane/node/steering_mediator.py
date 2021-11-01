#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

'''
Currently, this mediator only starts the required service for emotiv in steering control
'''
def empty_handler(self, req):
    return True

if __name__ == "__main__":
    rospy.init_node('steering_mediator')
    rospy.wait_for_message("/amcl_pose",PoseWithCovarianceStamped)
    rospy.loginfo('[{0}]: /amcl_pose received, start emotiv service'.format(rospy.get_name()))
    emotiv_srv = rospy.Service("start_emotiv_srv", Empty, empty_handler)

    rospy.spin()
