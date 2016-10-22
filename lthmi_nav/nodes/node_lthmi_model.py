#!/usr/bin/env python

import rospy

from threading import Thread
from numpy import array, eye, random, sqrt

from lthmi_nav.msg import Command

from lthmi_nav.SynchronizableNode import SynchronizableNode

class StochHmiModel (SynchronizableNode):
    def __init__(self):
        SynchronizableNode.__init__(self)
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
                
                
        #self.interface_matrix_thresholds = list(interface_matrix)
        #for row in self.interface_matrix_thresholds:
            #thresh = 0.0
            #for k in xrange(len(row)):
                #row[k] = thresh+row[k]
                #thresh = row[k]
        
    
    def start(self, req):
        self.cmd_detected = Command()
        #rospy.loginfo("%s: 1>>>>>>>>>>>>>>> SEQ==%d", rospy.get_name(), self.cmd_detected.header.seq)
        self.sub = rospy.Subscriber('/cmd_intended', Command, self.cmdIntendedCallback)
        self.pub = rospy.Publisher('/cmd_detected', Command, queue_size=10)
    
    def stop(self):
        self.sub.shutdown()
        self.pub.shutdown()
    
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
            self.pub.publish(self.cmd_detected)
            rospy.loginfo("%s: published detected command, intended=%d, detected=%d (SEQ==%d)", rospy.get_name(), self.cmd_intended, self.cmd_detected.cmd, self.cmd_detected.header.seq)
    
    def cmdIntendedCallback(self, msg):
        is_fisrt_msg = (self.cmd_intended is None)
        self.cmd_intended = msg.cmd
        if self.period==0.0:
            self.cmd_detected.header.stamp = rospy.Time.now()
            self.cmd_detected.cmd = self.randomize(self.cmd_intended)
            if self.delay==0.0:
                raw_input("Press Enter to continue\n")
            elif self.delay>0.0:
                rospy.loginfo("%s: sleeping %fsec" %(rospy.get_name(), self.delay))
                rospy.sleep(self.delay) 
            #rospy.loginfo("%s: 2>>>>>>>>>>>>>>> SEQ==%d", rospy.get_name(), self.cmd_detected.header.seq)
            self.pub.publish(self.cmd_detected)
            #rospy.loginfo("%s: 3>>>>>>>>>>>>>>> SEQ==%d", rospy.get_name(), self.cmd_detected.header.seq)
            rospy.loginfo("%s: published detected command, intended=%d, detected=%d (SEQ==%d)", rospy.get_name(), self.cmd_intended, self.cmd_detected.cmd, self.cmd_detected.header.seq)
        elif is_fisrt_msg:
            Thread(target=self.publishCmdDetected).start()
    

if __name__=="__main__":
    rospy.init_node('stoch_hmi_model')
    hm = StochHmiModel()
    hm.run()







#self.cmd_intended = None

#def intendedCommandReceived(msg):
    #global self.cmd_intended, period, delay
    #is_fisrt_msg = (self.cmd_intended is None)
    #self.cmd_intended = msg.cmd
    #if period==0.0:
        #msg2pub = Command()
        #msg2pub.cmd = randomize(interface_matrix_thresholds, self.cmd_intended)
        #if delay==0.0:
            #raw_input("Press Enter to continue\n")
        #elif delay>0.0:
            #rospy.loginfo("stochastic_hmi_model: speeping %fsec", delay)
            #rospy.sleep(delay) 
        #pub.publish(msg2pub)
        #rospy.loginfo("stochastic_hmi_model: published detected command, intended=%d, detected=%d", self.cmd_intended, msg2pub.cmd)
    #elif is_fisrt_msg:
        #Thread(target=publishDetectedCmd, args=[pub, period, interface_matrix_thresholds]).start()
    

#def randomize(interface_matrix_thresholds, intended):
    #""" randomize output according to interfce matrix """
    #r = random.random()
    #thresholds = interface_matrix_thresholds[intended]
    #for index, thr in enumerate(thresholds):
        #if r<thr:
            #detected = index
            #break
    #return detected
    
#def publishDetectedCmd(pub, period, interface_matrix_thresholds):
    #while(not rospy.is_shutdown()):
        #rospy.sleep(period) 
        #msg2pub = Command()
        #msg2pub.cmd = randomize(interface_matrix_thresholds, self.cmd_intended)
        #pub.publish(msg2pub)
        #rospy.loginfo("stochastic_hmi_model: published detected command, intended=%d, detected=%d", self.cmd_intended, msg2pub.cmd)
    
#if __name__=="__main__":
    #global pub, interface_matrix, period, delay
    #sub = rospy.Subscriber('/cmd_intended', Command, intendedCommandReceived)
    #pub = rospy.Publisher('/cmd_detected', Command, queue_size=10)
    #rospy.init_node('stochastic_hmi_model')
    
    ##read parameters
    #n =rospy.get_param('~number_of_commands', 4)
    #interface_matrix_tmp = rospy.get_param('~interface_matrix', [])
    #interface_matrix = array(interface_matrix_tmp) if len(interface_matrix_tmp)>0 else eye(n)
    #period = rospy.get_param('~period', 0.0)
    #delay  = rospy.get_param('~delay', 0.0)
 
    ## prepare additional vars
    #interface_matrix_thresholds = interface_matrix.copy()
    #for row in interface_matrix_thresholds:
        #thresh = 0.0
        #for k in xrange(len(row)):
            #row[k] = thresh+row[k]
            #thresh = row[k]
    
    #rospy.logwarn("\n=========INTERFACE MATRIX:====== \n" + str(interface_matrix))
    #rospy.spin()
