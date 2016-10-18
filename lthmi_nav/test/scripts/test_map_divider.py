#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from lthmi_nav.msg import IntMap
from lthmi_nav.msg import FloatMap

from datetime import datetime
import random
import math

from lthmi_nav.SyncingNode import SyncingNode


class MapDividerTester (SyncingNode):
    def __init__(self):
        SyncingNode.__init__(self, publishMap=True)
        self.cfg.update({
            'map_file':      rospy.get_param('~map'),
            'resolution':    rospy.get_param('~resolution', 0.1),
            'n_experiments': rospy.get_param('~n_experiments', 1),
            'n_poses':       rospy.get_param('~n_poses', 1),
            'pose_x':        rospy.get_param('~pose_x', 0), #0 means random pose
            'pose_y':        rospy.get_param('~pose_y', 0),
            'pdf_seed':      rospy.get_param('~pdf_seed', -1), 
            'delay':         rospy.get_param('~delay', -1.0) #negative means forever
            #'period':        rospy.get_param('~period', -1.0) #0.0 means wait forever
        })
        
        self.pdf_publisher   = rospy.Publisher('/pdf', FloatMap, queue_size=1, latch=True) #, latch=False)
        self.pose_publisher  = rospy.Publisher('/pose_optimal', PoseStamped, queue_size=1, latch=True)#, latch=False)
        self.map_divided_sub = rospy.Subscriber('/map_divided', IntMap, self.mapDividedCallback)
        self.exps_left       = self.cfg['n_experiments']
        self.poses_left      = 0
        random.seed(datetime.now())
    
    def publishNextPose(self):
        if self.poses_left==0:
            if self.exps_left==0:
                exit(0)
            self.runExperiment(self.cfg['map_file'], self.cfg['resolution'], Pose())
            self.exps_left -= 1
        rospy.loginfo("%s: ============ testing next pose ===========" % (rospy.get_name()))
        self.poses_left -= 1
        self.publishPdf()
        self.publishPose()
    
    def runTester(self):
        for k in range(self.cfg['n_experiments']): #map_file in maps:
            self.runExperiment(self.cfg['map_file'], self.cfg['resolution'], Pose())
            for k in range(self.cfg['n_poses']):
                self.publishPdf()
                self.publishPose()
                if self.cfg['period']==0.0:
                    rospy.spin()
                else:
                    rospy.sleep(self.cfg['period'])
                    
    def publishPdf(self):
        pdf = FloatMap()
        pdf.header.frame_id = "/map"
        pdf.info.width  = self.grid.width+1
        pdf.info.height = self.grid.height+1
        pdf.info.resolution = self.cfg['resolution']
        pdf.info.origin.position.x = -0.5*self.cfg['resolution']
        pdf.info.origin.position.y = -0.5*self.cfg['resolution']
        #pdf.info.origin.position.z = -0.001
        pdf.data = [-1.0 for x in range(pdf.info.width*pdf.info.height)]
        
        k = self.cfg['pdf_seed']#20 #random.randint(0, pdf.info.width)
        if k<0:
            k = random.randint(0, pdf.info.width)
        total = 0.0
        for x in range(1,pdf.info.width-1):
            for y in range(1,pdf.info.height-1):
                if self.grid.isVertexUnblocked(x,y):
                    p = 2.0+ math.cos(  math.sqrt(  ((x-k)/30.0)**2 + ((y-k)/21.0)**2)  )
                    total +=p 
                    pdf.data[x+ y*pdf.info.width] = p
        pdf.data = [p/total if p>0.0 else p for p in pdf.data]
        self.pdf = pdf
        self.pdf_publisher.publish(pdf) 
        rospy.loginfo("%s: published pdf resolution=(%d,%d), cell_size=%f, pdf_seed=%d" % (rospy.get_name(), pdf.info.width, pdf.info.height, pdf.info.resolution, k))


    def publishPose(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        if self.cfg['pose_x'] != 0:
            optimalVertex = [self.cfg['pose_x'], self.cfg['pose_y']]
        else:
            optimalVertex = self.grid.genRandUnblockedVertex() #[114,22]
        pose.pose.position.x = optimalVertex[0]*self.cfg['resolution']
        pose.pose.position.y = optimalVertex[1]*self.cfg['resolution']
        pose.header.frame_id="/map"
        self.pose_publisher.publish(pose)
        rospy.loginfo("%s: published /pose_optimal vertex=(%d,%d), pose=(%f,%f)" % (rospy.get_name(), optimalVertex[0], optimalVertex[1], pose.pose.position.x, pose.pose.position.y))

    def mapDividedCallback(self, msg):
        for x in range(msg.info.width):
            for y in range(msg.info.height):
                reg = msg.data[x + y*msg.info.width]
                if reg==255 and self.pdf.data[x + y*msg.info.width]>=0:
                    rospy.logerr("%s: ERROR: divided map contains a vertex at [%d,%d] without a region assigned" % (rospy.get_name(),x,y))
                    exit(1)
        rospy.loginfo("%s: /divided_map recieived and checked for unussigned vertices (all fine)." % (rospy.get_name()))
        
        if self.cfg['delay']>0:
            rospy.sleep(self.cfg['delay'])
        self.publishNextPose()
        
if __name__=="__main__":
    rospy.init_node('test_map_divider')
    mdt = MapDividerTester()
    mdt.publishNextPose()
    rospy.spin()
