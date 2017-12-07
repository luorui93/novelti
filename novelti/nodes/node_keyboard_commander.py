#!/usr/bin/env python

import rospy
import tf

import sys
import select
import termios
import tty

from numpy import array, eye, random

from novelti.msg import Command
from std_srvs.srv import Empty
from novelti.SynchronizableNode import SynchronizableNode


class KeyboardCommander (SynchronizableNode):
    def __init__(self):
        self.srv_name = rospy.get_param('~new_goal')
        
        # read parameters
        self.key_mappings = rospy.get_param('~key_mappings', ['0', '1', '2', '3'])
        self.new_goal_key = rospy.get_param('~new_goal_key', 'r')
        SynchronizableNode.__init__(self)

    def start(self, req):
        rospy.wait_for_service(self.srv_name)
        self.new_goal_srv = rospy.ServiceProxy(self.srv_name, Empty)
        self.pub_cmd_intended = rospy.Publisher('/cmd_intended', Command, queue_size=1)
        # self.map_divided_sub   = rospy.Subscriber('/map_divided', IntMap, self.mapDividedCallback)
        # self.sub_pose_intended = rospy.Subscriber('/pose_intended', PoseStamped, self.poseIntendedCallback)

    def stop(self):
        # self.map_divided_sub.unregister()
        # self.sub_pose_intended.unregister()
        self.pub_cmd_intended.unregister()


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("keyboard_commander")
    kc = KeyboardCommander()

    key_mappings_dict = {}
    for index, key in enumerate(kc.key_mappings):
        key_mappings_dict[key] = index

    rospy.loginfo("=====KEY MAPPINGS: %s, new goal command: '%s'" %
                  (str(key_mappings_dict), kc.new_goal_key))

    try:
        while(1):
            key = getKey()
            if key in kc.key_mappings:
                intended = key_mappings_dict[key]
                msg = Command()
                msg.cmd = intended
                kc.pub_cmd_intended.publish(msg)
            elif key == kc.new_goal_key:
                try:
                    resp = kc.new_goal_srv()
                except rospy.ServiceException, e:
                    rospy.logerr("Service call failed: %s" % e)
                    exit(1)
                rospy.loginfo("%s: Initiating a new goal" % (rospy.get_name()))
            else:
                #x = 0
                #th = 0
                if (key == '\x03'):
                    break
    except BaseException as e:
        print e
        rospy.logerr(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    kc.run()