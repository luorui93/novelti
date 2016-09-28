#!/usr/bin/env python

import rospy, tf

from threading import Thread
from numpy import array, eye, random

from low_throughput_hmi.msg import Command

cur_intended_cmd = None

def intendedCommandReceived(msg):
    global cur_intended_cmd, period, delay
    is_fisrt_msg = (cur_intended_cmd is None)
    cur_intended_cmd = msg.cmd
    if period==0.0:
        msg2pub = Command()
        msg2pub.cmd = randomize(interface_matrix_thresholds, cur_intended_cmd)
        if delay==0.0:
            raw_input("Press Enter to continue\n")
        elif delay>0.0:
            rospy.loginfo("stochastic_hmi_model: speeping %fsec", delay)
            rospy.sleep(delay) 
        pub.publish(msg2pub)
        rospy.loginfo("stochastic_hmi_model: published detected command, intended=%d, detected=%d", cur_intended_cmd, msg2pub.cmd)
    elif is_fisrt_msg:
        Thread(target=publishDetectedCmd, args=[pub, period, interface_matrix_thresholds]).start()
    

def randomize(interface_matrix_thresholds, intended):
    """ randomize output according to interfce matrix """
    r = random.random()
    thresholds = interface_matrix_thresholds[intended]
    for index, thr in enumerate(thresholds):
        if r<thr:
            detected = index
            break
    return detected
    
def publishDetectedCmd(pub, period, interface_matrix_thresholds):
    while(not rospy.is_shutdown()):
        rospy.sleep(period) 
        msg2pub = Command()
        msg2pub.cmd = randomize(interface_matrix_thresholds, cur_intended_cmd)
        pub.publish(msg2pub)
        rospy.loginfo("stochastic_hmi_model: published detected command, intended=%d, detected=%d", cur_intended_cmd, msg2pub.cmd)
    
if __name__=="__main__":
    global pub, interface_matrix, period, delay
    sub = rospy.Subscriber('/intended_command', Command, intendedCommandReceived)
    pub = rospy.Publisher('/detected_command', Command)
    rospy.init_node('stochastic_hmi_model')
    
    #read parameters
    n =rospy.get_param('~number_of_commands', 4)
    interface_matrix_tmp = rospy.get_param('~interface_matrix', [])
    interface_matrix = array(interface_matrix_tmp) if len(interface_matrix_tmp)>0 else eye(n)
    period = rospy.get_param('~period', 0.0)
    delay  = rospy.get_param('~delay', 0.0)
 
    # prepare additional vars
    interface_matrix_thresholds = interface_matrix.copy()
    for row in interface_matrix_thresholds:
        thresh = 0.0
        for k in xrange(len(row)):
            row[k] = thresh+row[k]
            thresh = row[k]
    
    rospy.logwarn("\n=========INTERFACE MATRIX:====== \n" + str(interface_matrix))
    rospy.spin()
