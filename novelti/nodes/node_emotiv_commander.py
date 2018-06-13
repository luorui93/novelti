#!/usr/bin/env python
"""                                      
                                     Init():
  runExperiment():                    Provide two services
   Call StartSrv()                    StartSrv and StopSrv

+--------------------+            +---------------------+
|                    |            |                     |
|  SyncingNode       +----------->+  SynchronizableNode +---------------------------+
|                    | StartSrv() |                     |                           |
|                    |            |                     |                           |
+-----+--------------+            +-------+------+------+                           |
      ^                                   |      ^                                  |
      | wait for StartSrv()       start() |      | Call Init()              start() |
      | Call runExperiment()              v      |                                  v
+-----+--------------+            +-------+------+------+                    +------+-------------+
|                    |            |                     | new_goal_key       |                    |
|  Experimentator /  |            |   EmotivCommander   | detected           |  Novelti /         |
|  Mediator          |            |                     +------------------->+  InferenceUnit     |
|                    |            |                     | call "new_goal"    |                    |
+--------------------+            +---------------------+ srvNewGoal()       +--------------------+
                                     start():                                   start():
                                      wait for "new_goal" service                advertise "new_goal" service
                                      init "new_goal" service client
 """
import math
import rospy

from novelti.SynchronizableNode import SynchronizableNode
from std_srvs.srv import Empty
from novelti.msg import Command

class EmotivCommander (SynchronizableNode):
    def __init__(self):
        self.srv_name = rospy.get_param('~new_goal_srv')
        self.new_goal_key = rospy.get_param('~new_goal_key', '4')
        #this __init__ will advertise two services: start/stop
        self.status = "WAIT_FOR_NEW_GOAL"
        SynchronizableNode.__init__(self)

    def start(self, req):
        rospy.wait_for_service(self.srv_name)
        self.new_goal_srv_client = rospy.ServiceProxy(self.srv_name,Empty)
        self.pub_cmd_detected = rospy.Publisher('/cmd_detected', Command, queue_size=1)
        self.sub_emotiv_cmd = rospy.Subscriber('/emotiv_cmd',Command,self.emotivCmdCallback)

    def emotivCmdCallback(self,msg):
        if msg.cmd == self.new_goal_key:
            try:
                resp = self.new_goal_srv_client()
                self.status = "NEW_GOAL_STARTED"
            except rospy.ServiceException, e:
                rospy.logerr("%s service call failed: %s" % (self.srv_name,e))
                exit(1)
        else :
            if self.status == "NEW_GOAL_STARTED":
                self.pub_cmd_detected.publish(msg)

if __name__ == "__main__" :
    rospy.init_node("sync_emotiv_commander") 
    ec = EmotivCommander()
    rospy.spin()
