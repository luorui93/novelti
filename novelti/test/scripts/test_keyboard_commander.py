#!/usr/bin/env python

import rospy

from numpy import array

from novelti.msg import Command


tested_vector = []
cmd_counters = []
total = 0

def command_callback(msg):
    detected = msg.cmd
    global cmd_counters, tested_vector, cmd_counters, total
    cmd_counters[detected] +=1
    total += 1
    rospy.logwarn("Expected probabilistic vector: " + str(tested_vector))
    rospy.logwarn("  Actual probabilistic vector: " + str(array(cmd_counters, dtype='Float32')/total))
    rospy.logwarn("                     Counters: " + str(cmd_counters))


if __name__=="__main__":
    rospy.init_node('keyboard_commander_test')
    
    #read parameters
    interface_matrix = array(rospy.get_param('~interface_matrix', []))
    cmd_counters =[0]*len(interface_matrix)
    tested_cmd =rospy.get_param('~tested_cmd', 0)
    #delay =rospy.get_param('~delay', 0.0)
    #use_latest =rospy.get_param('~use_latest', True)
 
    # prepare additional vars
    tested_vector = interface_matrix[tested_cmd]
    
    sub = rospy.Subscriber('/cmd_detected', Command, command_callback)

    rospy.spin()