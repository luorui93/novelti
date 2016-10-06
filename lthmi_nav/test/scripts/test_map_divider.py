#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import *
from lthmi_nav.msg import IntMap
from lthmi_nav.msg import FloatMap
from lthmi_nav.srv import StartExperiment

from lthmi_nav.MapTools import GridMap
import random
import math


def wait_for_srvs():
    rospy.wait_for_service('/rviz/reload_shaders')
    srv_path = '/map_divider/start'
    rospy.wait_for_service(srv_path)
    return rospy.ServiceProxy(srv_path, StartExperiment)

def start(div_srv, grid, resolution):
    seq = 0
    stamp = rospy.Time.now()
    name = "qwe"
    mapa = IntMap()
    mapa.header.frame_id = "/map"
    mapa.data = [(0 if v==grid.FREE else 1) for v in grid.data]
    mapa.info.width = grid.width
    mapa.info.height = grid.height
    mapa.info.resolution = resolution
    init_pose = Pose()
    try:
        div_srv(seq, stamp, name, mapa, init_pose)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def gen_map(grid, resolution):
    m = IntMap()
    m.header.frame_id = "/map"
    m.info.width  = grid.width
    m.info.height = grid.height
    m.info.resolution = resolution
    m.info.origin.position.x = 0.5*resolution
    m.info.origin.position.y = 0.5*resolution
    #m.info.origin.position.z = -0.001
    m.data = [255 if grid.isFree(k) else 254 for k in range(len(grid.data))] #255 - transparent, 254 - black
    return m
    
def gen_pdf(grid, resolution):
    pdf = FloatMap()
    pdf.header.frame_id = "/map"
    pdf.info.width  = grid.width+1
    pdf.info.height = grid.height+1
    pdf.info.resolution = resolution
    #pdf.info.origin.position.z = -0.001
    pdf.data = [-1.0 for x in range(pdf.info.width*pdf.info.height)]

    k = random.randint(0, pdf.info.width)
    total = 0.0
    for x in range(1,pdf.info.width-1):
        for y in range(1,pdf.info.height-1):
            if grid.isVertexUnblocked(x,y):
                p = 2.0+ math.cos(  math.sqrt(  ((x-k)/30.0)**2 + ((y-k)/21.0)**2)  )
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
    map_file = rospy.get_param('~map')
    resolution = rospy.get_param('~resolution', 0.1)
    n_experiments = rospy.get_param('~n_experiments', 1)
    n_poses = rospy.get_param('~n_poses', 1)
    sleep_forever = rospy.get_param('~sleep_forever', True)
    
    pdf_publisher  = rospy.Publisher('/pdf', FloatMap, queue_size=1, latch=True) #, latch=False)
    pose_publisher = rospy.Publisher('/pose_optimal', PoseStamped, queue_size=1, latch=True)#, latch=False)
    map_publisher  = rospy.Publisher('/map', IntMap, queue_size=1, latch=True)#, latch=False)
    
    rate = rospy.Rate(0.1) 
    srv = wait_for_srvs()
    for k in range(n_experiments): #map_file in maps:
        grid = GridMap.fromText(open(map_file, 'r'))
        start(srv, grid, resolution)
        for k in range(n_poses):
            map_publisher.publish(gen_map(grid, resolution))
            
            pdf = gen_pdf(grid, resolution)
            pdf_publisher.publish(pdf) 
            rospy.loginfo("test_map_divider: published pdf (%d,%d), resolution=%f" % (pdf.info.width, pdf.info.height, pdf.info.resolution))
            
            pose_publisher.publish(gen_pose(grid, resolution))
            rospy.sleep(5)
    rospy.spin()