#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped
from novelti.msg import IntMap
from novelti.msg import Command

from novelti.SynchronizableNode import SynchronizableNode
from std_srvs.srv import Empty

class HumanModel (SynchronizableNode):
    def __init__(self):
        self.srv_name = '/inference_unit/new_goal'
        SynchronizableNode.__init__(self)
    
    def start(self, req):
        rospy.wait_for_service(self.srv_name)
        self.new_goal_srv = rospy.ServiceProxy(self.srv_name, Empty)
        self.cmd_intended = Command()
        self.pub_cmd_intended  = rospy.Publisher('/cmd_intended', Command, queue_size=1, latch=True)#, latch=False)
        self.map_divided_sub   = rospy.Subscriber('/map_divided', IntMap, self.mapDividedCallback)
        self.sub_pose_intended = rospy.Subscriber('/pose_intended', PoseStamped, self.poseIntendedCallback)
    
    def stop(self):
        self.map_divided_sub.unregister()
        self.sub_pose_intended.unregister()
        self.pub_cmd_intended.unregister()
    
    def mapDividedCallback(self, msg):
        reg = msg.data[self.vertex_intended[0] + self.vertex_intended[1]*msg.info.width]
        self.cmd_intended.header.stamp = rospy.Time.now()
        self.cmd_intended.cmd = reg
        rospy.loginfo("%s: number of connections=%d" % (rospy.get_name(), self.pub_cmd_intended.get_num_connections()))
        self.pub_cmd_intended.publish(self.cmd_intended)
        rospy.loginfo("%s: /map_divided recieived (SEQ=%d), published /cmd_intended=%d (SEQ=%d)." % (rospy.get_name(), msg.header.seq, reg,  self.cmd_intended.header.seq))

    def poseIntendedCallback(self, msg):
        self.vertex_intended = self.pose2vertex(msg.pose)
        try:
            resp = self.new_goal_srv()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            exit(1)
        rospy.loginfo("%s: /pose_intended received, vertex=(%d,%d), pose=(%f,%f)." % (rospy.get_name(), self.vertex_intended[0], self.vertex_intended[1], msg.pose.position.x, msg.pose.position.y))

if __name__=="__main__":
    rospy.init_node('human_model')
    hm = HumanModel()
    hm.run()
