#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Pose
from lthmi_nav.srv import StartExperiment
from lthmi_nav.MapTools import GridMap
from lthmi_nav.msg import IntMap

class SyncingNode:
    def __init__(self, publishMap=False):
        self.cfg = {
            'synced_nodes':  rospy.get_param('~synced_nodes', []),
            'waited_srvs':   rospy.get_param('~waited_srvs', [])
        }
        self.seq = 0;
        self.waitForAll()
        if publishMap:
            self.map_publisher  = rospy.Publisher('/map', IntMap, queue_size=1, latch=True) if publishMap else Node

    def waitForAll(self):
        for srv_path in self.cfg['waited_srvs']:
            rospy.wait_for_service(srv_path)
        self.srvs = []
        for node_name in self.cfg['synced_nodes']:
            srv_path = node_name+'/start'
            rospy.wait_for_service(srv_path)
            self.srvs.append(rospy.ServiceProxy(srv_path, StartExperiment))
    
    def publishMap(self, resolution):
        m = IntMap()
        m.header.frame_id = "/map"
        m.info.width  = self.grid.width
        m.info.height = self.grid.height
        m.info.resolution = resolution
        #m.info.origin.position.x = 0.5*resolution
        #m.info.origin.position.y = 0.5*resolution
        #m.info.origin.position.z = -0.001
        m.data = [255 if self.grid.isFree(k) else 254 for k in range(len(self.grid.data))] #255 - transparent, 254 - black
        self.map_publisher.publish(m)
    
    def runExperiment(self, map_file, resolution, init_pose):
        self.grid = GridMap.fromText(open(map_file, 'r'))
        if self.map_publisher is not None:
            self.publishMap(resolution)
        stamp = rospy.Time.now()
        name = "Experiment %d" % self.seq
        mapa = IntMap()
        mapa.header.frame_id = "/map"
        mapa.data = [(0 if v==self.grid.FREE else 1) for v in self.grid.data]
        mapa.info.width = self.grid.width
        mapa.info.height = self.grid.height
        mapa.info.resolution = resolution
        rospy.loginfo("============================== Starting experiment '%s' ========================================="%name)
        for srv in self.srvs:
            try:
                srv(self.seq, stamp, name, mapa, init_pose)
            except rospy.ServiceException, e:
                print "Service call to node %s failed: %s"%e
        self.seq += 1
                
