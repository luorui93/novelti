#!/usr/bin/env python

import rospy

from novelti.msg import IntMap

#for map_divided
NOT_REACHABLE = -1
N_REGIONS = 5

if __name__=="__main__":
    pub = rospy.Publisher('/map_divided', IntMap, queue_size=2)
    rospy.init_node('test_rviz_int_map')
    #map dimensions
    mw = 200
    mh = 150
    
    w  = mw+1
    h  = mh+1
    mapa = IntMap()
    mapa.info.resolution = 0.1
    mapa.info.width = w
    mapa.info.height = h
    mapa.data = [NOT_REACHABLE]*w*h;
    
    #regions as stripes
    for x in range(1,mw):
        for y in range(1,mh):
            mapa.data[y*w + x] = x / (w/N_REGIONS)
            
    #emaulate unreachable rect area in the center
    for x in range(w/3,2*w/3):
        for y in range(h/3,2*h/3):
            mapa.data[y*w + x] = NOT_REACHABLE

    rate = rospy.Rate(2) 
    while not rospy.is_shutdown():
        pub.publish(mapa)
        rate.sleep()
    
    rospy.spin()