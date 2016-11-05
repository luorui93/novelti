#!/usr/bin/env python

import rospy

from threading import Thread
from numpy import array, eye, random, sqrt

from lthmi_nav.msg import Command

from lthmi_nav.SynchronizableNode import SynchronizableNode

class StochHmiModel (SynchronizableNode):
    def __init__(self):
        self.cmd_intended = None
        #read parameters
        interface_matrix = rospy.get_param('~interface_matrix', [])
        rospy.loginfo("\n=========INTERFACE MATRIX:====== \n" + str(interface_matrix))
        self.period = rospy.get_param('~period', 0.0)
        self.delay  = rospy.get_param('~delay', 0.0)
        n = int(sqrt(len(interface_matrix)))

        k=0
        self.interface_matrix_thresholds = []
        for r in range(n):
            self.interface_matrix_thresholds.append([])
            for c in range(n):
                self.interface_matrix_thresholds[r].append(interface_matrix[k])
                k += 1
        #rospy.loginfo("\n=========INTERFACE TTHR:====== \n" + str(self.interface_matrix_thresholds))
        # prepare additional vars
        for row in self.interface_matrix_thresholds:
            #rospy.loginfo("======= %s====%d\n" % (str(row), len(row)))
            thresh = 0.0
            for k in xrange(n):
                row[k] = thresh+row[k]
                thresh = row[k]
        SynchronizableNode.__init__(self)
    
    def start(self, req):
        self.cmd_detected = Command()
        self.sub = rospy.Subscriber('/cmd_intended', Command, self.cmdIntendedCallback)
        self.pub = rospy.Publisher('/cmd_detected', Command, queue_size=1, latch=True)
    
    def stop(self):
        self.sub.unregister()
        self.pub.unregister()
    
    def randomize(self, intended):
        """ randomize output according to interfce matrix """
        r = random.random()
        thresholds = self.interface_matrix_thresholds[intended]
        for index, thr in enumerate(thresholds):
            if r<thr:
                detected = index
                break
        return detected
    
    
    def publishCmdDetected(self):
        while(not rospy.is_shutdown()):
            rospy.sleep(self.period) 
            self.cmd_detected.header.stamp = rospy.Time.now()
            self.cmd_detected.cmd = self.randomize(self.cmd_intended)
            rospy.loginfo("%s: number of connections=%d" % (rospy.get_name(), self.pub.get_num_connections()))
            self.pub.publish(self.cmd_detected)
            rospy.loginfo("%s: published detected command, intended=%d, detected=%d (SEQ==%d)", rospy.get_name(), self.cmd_intended, self.cmd_detected.cmd, self.cmd_detected.header.seq)
    
    def cmdIntendedCallback(self, msg):
        is_fisrt_msg = (self.cmd_intended is None)
        self.cmd_intended = msg.cmd
        if self.period==0.0:
            self.cmd_detected.cmd = self.randomize(self.cmd_intended)
            if self.delay==0.0:
                raw_input("Press Enter to continue\n")
            elif self.delay>0.0:
                rospy.loginfo("%s: sleeping %fsec" %(rospy.get_name(), self.delay))
                rospy.sleep(self.delay) 
            self.cmd_detected.header.stamp = rospy.Time.now()
            self.pub.publish(self.cmd_detected)
            rospy.loginfo("%s: published detected command, intended=%d, detected=%d (SEQ==%d)", rospy.get_name(), self.cmd_intended, self.cmd_detected.cmd, self.cmd_detected.header.seq)
        elif is_fisrt_msg:
            Thread(target=self.publishCmdDetected).start()
    

if __name__=="__main__":
    rospy.init_node('stoch_hmi_model')
    hm = StochHmiModel()
    hm.run()
