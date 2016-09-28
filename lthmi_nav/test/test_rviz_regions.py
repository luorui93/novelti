#!/usr/bin/env python

import rospy

from nav_msgs.msg import OccupancyGrid

if __name__=="__main__":
    pub = rospy.Publisher('/divided_map', OccupancyGrid)
    rospy.init_node('inference_module')
    
    pdf = OccupancyGrid()
    pdf.info.resolution = 0.1
    pdf.info.width = 64
    pdf.info.height = 64
    pdf.data = [-10]*64*8 + [-1]*64*8 + [0]*64*8 + [1]*64*8 + [2]*64*8 + [3]*64*8 + [4]*64*8 + [5]*64*8 
    
    rate = rospy.Rate(2) 
    while not rospy.is_shutdown():
        pub.publish(pdf)
        rate.sleep()
    
    rospy.spin()
