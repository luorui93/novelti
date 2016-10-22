#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('chatter', PoseStamped, queue_size=1000)
    rospy.init_node('talker', anonymous=True)
    rospy.sleep(10) # just to ensure we don't miss any messages with rostopic echo
    rate = rospy.Rate(1)
    msg = PoseStamped()
    #msg.header.seq = -1
    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.loginfo("seq=%d" % msg.header.seq)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass