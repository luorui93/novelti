#!/usr/bin/env python

import rospy, tf

import sys, select, termios, tty

from numpy import array, eye, random

from lthmi_nav.msg import Command

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    pub = rospy.Publisher('/cmd_intended', Command, queue_size=10)
    rospy.init_node('keyboard_commander')
    
    #read parameters
    key_mappings  = rospy.get_param('~key_mappings', ['0','1','2','3'])
 
    key_mappings_dict ={}
    for index, key in enumerate(key_mappings):
        key_mappings_dict[key] = index
    
    rospy.logwarn("=====KEY MAPPINGS:===== " + str(key_mappings_dict))
    
    
    try:
        while(1):
            key = getKey()
            if key in key_mappings:
                intended = key_mappings_dict[key]
                msg = Command()
                msg.cmd = intended
                pub.publish(msg)
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