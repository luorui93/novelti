#!/usr/bin/env python

import rospy

from nav_msgs.msg import OccupancyGrid

if __name__=="__main__":
    pub = rospy.Publisher('/pdf', OccupancyGrid)
    rospy.init_node('inference_module')
    
    pdf = OccupancyGrid()
    pdf.info.resolution = 0.1
    pdf.info.width = 100
    pdf.info.height = 100
    pdf.data = [0]*100*100;
    
    for k in range(0,100):
        pdf.data[k*100:(k+1)*100-1] = [k]*100
        
    
    rate = rospy.Rate(2) 
    while not rospy.is_shutdown():
        pub.publish(pdf)
        rate.sleep()
    
    rospy.spin()