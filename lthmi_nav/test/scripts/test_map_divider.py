#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import *
from lthmi_nav.msg import IntMap
from lthmi_nav.msg import FloatMap
from lthmi_nav.srv import StartExperiment

import lthmi_nav.MapTools


def wait_for_srvs():
    rospy.wait_for_service('/rviz/reload_shaders')
    srv_path = '/map_divider/start'
    rospy.wait_for_service(srv_path)
    return rospy.ServiceProxy(srv_path, StartExperiment)

def start(div_srv, grid):
    seq = 0
    stamp = rospy.Time.now()
    name = "qwe"
    mapa = IntMap()
    mapa.header.frame_id = "/map"
    mapa.data = [(1 if v else 0) for v in grid.data]
    mapa.info.width = grid.width
    mapa.info.height = grid.height
    mapa.info.resolution = 0.1
    init_pose = Pose()
    try:
        div_srv(seq, stamp, name, mapa, init_pose)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def gen_pdf(grid, resolution):
    pdf = FloatMap()
    pdf.header.frame_id = "/map"
    pdf.info.width  = grid.width+1
    pdf.info.height = grid.height+1
    pdf.info.resolution = 0.1
    pdf.data = [-1.0 for x in range(pdf.info.width*pdf.info.height)]

    k = randint(0, pdf.info.width)
    total = 0.0
    for x in range(1,pdf.info.width-1):
        for y in range(1,pdf.info.height-1):
            if grid.isVertexUnblocked(x,y):
                p = 2.0+ math.cos(  math.sqrt(  ((x-k)/5.0)**2 + ((y-k)/3.0)**2)  )
                total +=p 
                pdf.data[x+ y*pdf.info.width] = p
    pdf.data = [p/total if p>0.0 else p for p in pdf.data]
    return pdf

def gen_pose(grid, resolution):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    v = grid.genRandUnblockedVertex()
    pose.pose.position.x = v[0]*resolution
    pose.pose.position.y = v[1]*resolution
    pose.header.frame_id="/map"
    return pose
    
if __name__=="__main__":
    rospy.init_node('test_map_divider')
    
    #import os
    #try:
        #user_paths = os.environ['PYTHONPATH'].split(os.pathsep)
        #rospy.logwarn("PATH: %s" % str(user_paths))
    #except KeyError:
        #user_paths = []
    
    
    maps = rospy.get_param('~maps', [])
    resolution = rospy.get_param('~resolution', 0.1)
    
    pdf_publisher  = rospy.Publisher('/pdf', FloatMap, queue_size=2) #, latch=False)
    pose_publisher = rospy.Publisher('/pose_optimal', PoseStamped, queue_size=2)#, latch=False)
    
    rate = rospy.Rate(1) 
    srv = wait_for_srvs()
    for map_file in maps:
        grid = MapTools.fromText(open(map_file, 'r'))
        start(srv, grid)
        for k in range(5):
            pdf_publisher.publish(gen_pdf(grid, resolution)) 
            pose_publisher.publish(gen_pose(grid, resolution))
            rospy.spinOnce()
            rate.sleep()
