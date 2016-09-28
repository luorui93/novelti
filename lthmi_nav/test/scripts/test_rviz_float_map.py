#!/usr/bin/env python

import rospy, math

from lthmi_nav.msg import FloatMap

if __name__=="__main__":
    pub = rospy.Publisher('/pdf', FloatMap)
    rospy.init_node('inference_module')
    
    pdf = FloatMap()
    pdf.info.resolution = 0.1
    pdf.info.width = 100
    pdf.info.height = 100
    pdf.data = [0]*100*100;
    

    k=50
    rate = rospy.Rate(2) 
    while not rospy.is_shutdown():
        for x in range(0,100):
            for y in range(0,100):
                pdf.data[x+y*100] = 2.0+ math.cos(  math.sqrt(  ((x-k)/5.0)**2 + ((y-k)/3.0)**2)  )
        k+=1
        if k==100:
            k=0
        pub.publish(pdf)
        rate.sleep()
    
    rospy.spin()