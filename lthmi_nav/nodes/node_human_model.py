#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped
from lthmi_nav.msg import IntMap
from lthmi_nav.msg import Command

from lthmi_nav.SynchronizableNode import SynchronizableNode
from std_srvs.srv import Empty

class HumanModel (SynchronizableNode):
    def __init__(self):
        SynchronizableNode.__init__(self)
        self.srv_name = '/inference_unit/new_goal'
    
    def start(self, req):
        rospy.wait_for_service(self.srv_name)
        self.new_goal_srv = rospy.ServiceProxy(self.srv_name, Empty)
        self.pub_cmd_intended  = rospy.Publisher('/cmd_intended', Command, queue_size=1, latch=True)#, latch=False)
        self.map_divided_sub   = rospy.Subscriber('/map_divided', IntMap, self.mapDividedCallback)
        self.sub_pose_intended = rospy.Subscriber('/pose_intended', PoseStamped, self.poseIntendedCallback)
    
    def stop(self):
        self.map_divided_sub.shutdown()
        self.sub_pose_intended.shutdown()
        self.pub_cmd_intended.shutdown()
    
    def mapDividedCallback(self, msg):
        reg = msg.data[self.vertex_intended[0] + self.vertex_intended[1]*msg.info.width]
        cmd_intended = Command()
        cmd_intended.header.stamp = rospy.Time.now()
        cmd_intended.cmd = reg
        self.pub_cmd_intended.publish(cmd_intended)
        rospy.loginfo("%s: /map_divided recieived, published /cmd_intended=%d." % (rospy.get_name(), reg))

    def poseIntendedCallback(self, msg):
        self.vertex_intended = self.pose2vertex(msg.pose)
        try:
            resp = self.new_goal_srv()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            exit(1)
        rospy.loginfo("%s: /pose_intended received, vertex=(%d,%d), pose=(%f,%f)." % (rospy.get_name(), self.vertex_intended[0], self.vertex_intended[1], pose.pose.position.x, pose.pose.position.y))

if __name__=="__main__":
    rospy.init_node('human_model')
    hm = HumanModel()
    hm.run()
