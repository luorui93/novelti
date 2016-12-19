#!/usr/bin/env python

import rospy, tf

import sys, select, termios, tty

from numpy import array, eye, random

from lthmi_nav.msg import Command
from std_srvs.srv import Empty

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def newGoal(new_goal_srv):
    try:
        resp = new_goal_srv()
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)
        exit(1)
    rospy.loginfo("%s: Initiating a new goal" % (rospy.get_name()))


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('keyboard_commander')
    pub = rospy.Publisher('/cmd_intended', Command, queue_size=10)

    srv_name = '/inference_unit/new_goal'
    rospy.wait_for_service(srv_name)
    new_goal_srv = rospy.ServiceProxy(srv_name, Empty)
    
    #read parameters
    key_mappings  = rospy.get_param('~key_mappings', ['0','1','2','3'])
    new_goal_key  = rospy.get_param('~new_goal_key', 'r')
 
    key_mappings_dict ={}
    for index, key in enumerate(key_mappings):
        key_mappings_dict[key] = index
    
    rospy.logwarn("=====KEY MAPPINGS: %s, new goal command: '%s'"  % (str(key_mappings_dict), new_goal_key))
    
    try:
        while(1):
            key = getKey()
            if key in key_mappings:
                intended = key_mappings_dict[key]
                msg = Command()
                msg.cmd = intended
                pub.publish(msg)
            elif key==new_goal_key:
                newGoal(new_goal_srv)
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