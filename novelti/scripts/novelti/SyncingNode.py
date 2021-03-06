#!/usr/bin/env python
from math import pi

import rospy
import tf_conversions

from geometry_msgs.msg import Pose, Quaternion
from novelti.srv import StartExperiment
from novelti.MapTools import GridMap
from novelti.msg import IntMap
from std_srvs.srv import Empty

class SyncingNode:
    def __init__(self, publishMap=False):
        self.cfg = {
            'synced_nodes':  rospy.get_param('~synced_nodes', []),
            'waited_srvs':   rospy.get_param('~waited_srvs', [])
        }
        self.seq = 0;
        self.ranBefore = False
        self.waitForAll()
        if publishMap:
            self.map_publisher  = rospy.Publisher('/map', IntMap, queue_size=1, latch=True) if publishMap else Node

    def waitForAll(self):
        #rospy.loginfo("Waiting for all synced nodes %s and services %s to start........." % (self.cfg['synced_nodes'], self.cfg['waited_srvs']))
        for srv_path in self.cfg['waited_srvs']:
            rospy.loginfo("%s: waiting for %s" % (rospy.get_name(), srv_path))
            rospy.wait_for_service(srv_path)
        self.startSrvs = []
        self.stopSrvs = []
        for node_name in self.cfg['synced_nodes']:
            srv_path = node_name+'/start'
            rospy.loginfo("%s: waiting for %s" % (rospy.get_name(), srv_path))
            rospy.wait_for_service(srv_path)
            self.startSrvs.append(rospy.ServiceProxy(srv_path, StartExperiment))
            srv_path = node_name+'/stop'
            rospy.loginfo("%s: waiting for %s" % (rospy.get_name(), srv_path))
            rospy.wait_for_service(srv_path)
            self.stopSrvs.append(rospy.ServiceProxy(srv_path, Empty))
            
    
    def publishMap(self, resolution):
        m = IntMap()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "/map"
        m.info.width  = self.grid.width
        m.info.height = self.grid.height
        m.info.resolution = resolution
        #m.info.origin.position.x = 0.5*resolution
        #m.info.origin.position.y = 0.5*resolution
        #m.info.origin.position.z = -0.001
        m.data = [255 if self.grid.isFree(k) else 254 for k in range(len(self.grid.data))] #255 - transparent, 254 - black
        self.map_publisher.publish(m)
    
    def finishExperiment(self):
        for srv in self.stopSrvs:
            try:
                srv()
            except rospy.ServiceException, e:
                print "Service call to /stop failed: %s"%e
    
    def runExperiment(self, map_file, resolution, init_pose=None):
        if self.ranBefore:
            self.finishExperiment()
        self.ranBefore = True
        self.grid = GridMap.fromText(open(map_file, 'r'))
        if self.map_publisher is not None:
            self.publishMap(resolution)
        if init_pose is None:
            init_vx = self.grid.genRandUnblockedVertex()
            init_pose = self.vertex2pose(init_vx)
        stamp = rospy.Time.now()
        name = "Experiment %d" % self.seq
        mapa = IntMap()
        mapa.header.frame_id = "/map_inflated"
        mapa.data = [(0 if v==self.grid.FREE else 1) for v in self.grid.data]
        mapa.info.width = self.grid.width
        mapa.info.height = self.grid.height
        mapa.info.resolution = resolution
        # rospy.logwarn('%s'%(mapa.header.frame_id))
        rospy.loginfo("============================== Starting experiment '%s' ===   init_pose=(%f,%f) =========================="%(name, init_pose.position.x,init_pose.position.y))
        for srv in self.startSrvs:
            try:
                srv(self.seq, stamp, name, mapa, init_pose)
            except rospy.ServiceException, e:
                print "Service call to /node/start failed: %s"%e
        self.seq += 1
    
    def vertex2pose(self,vx):
        p = Pose()
        p.position.x = vx[0]*self.cfg['resolution']
        p.position.y = vx[1]*self.cfg['resolution']
        p.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, vx[2]))
        return p
