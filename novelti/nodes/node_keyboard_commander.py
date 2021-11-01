#!/usr/bin/env python

import rospy
import tf

import sys
import select
import termios
import tty

from numpy import array, eye, random

from novelti.msg import Command

import signal


class KeyboardCommander:
    def __init__(self):        
        # read parameters
        self.key_mappings = rospy.get_param('~key_mappings', ['0', '1', '2', '3'])
        self.pub_keyboard_cmd = rospy.Publisher('/keyboard_cmd_intended', Command, queue_size=1)

def getKey():
    global settings
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def exitGracefully(signum, frame):
    global settings
    rospy.logwarn("Caught SIGINT, gracefully dying...")
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    exit(0)

if __name__ == "__main__":
    global settings
    settings = None
    signal.signal(signal.SIGINT, exitGracefully)

    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("keyboard_commander")
    kc = KeyboardCommander()

    key_mappings_dict = {}
    for index, key in enumerate(kc.key_mappings):
        key_mappings_dict[key] = index

    rospy.loginfo("=====KEY MAPPINGS: %s" %
                  (str(key_mappings_dict)))

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key in kc.key_mappings:
                intended = key_mappings_dict[key]
                msg = Command()
                msg.cmd = intended
                kc.pub_keyboard_cmd.publish(msg)
            else:
                #\x03 is ctrl+c
                if (key == '\x03'):
                    break
    except BaseException as e:
        print e
        rospy.logerr(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)