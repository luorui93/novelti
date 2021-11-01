#!/usr/bin/env python

import rospy

from threading import Thread
from numpy import array, eye, random, sqrt

from novelti.msg import Command

class StochHmiModel:
    def __init__(self):
        self.cmd_intended = None
        #read parameters
        """if period!=0, then 
                the first cmd_detected message starts a thread that will publish /cmd_intended
                at constant period T=period
                the the delay parameter in this case is ignored 
            otherwise (period==0):
                then every time a cmd_detected received, the node sleeps for delay seconds
                and publishes cmd_intended calcualted from cmd_detected and interface matrix
        """        

        interface_matrix = rospy.get_param('~interface_matrix', [])
        rospy.loginfo("\n=========INTERFACE MATRIX:====== \n" + str(interface_matrix))
        self.period = rospy.get_param('~period', 0.0)
        self.default_cmd = rospy.get_param('~default_cmd', "prev")
        self.delay  = rospy.get_param('~delay', 0.0)
        self.n = int(sqrt(len(interface_matrix)))
        self.cmd_detected = Command()
        self.callbackRunning = False
        self.sub = rospy.Subscriber('/cmd_intended', Command, self.cmdIntendedCallback, queue_size=1, buff_size=10000)
        self.pub = rospy.Publisher('/cmd_detected', Command, queue_size=1, latch=True)
        k=0
        self.interface_matrix_thresholds = []
        for r in range(self.n):
            self.interface_matrix_thresholds.append([])
            for c in range(self.n):
                self.interface_matrix_thresholds[r].append(interface_matrix[k])
                k += 1
        #rospy.loginfo("\n=========INTERFACE TTHR:====== \n" + str(self.interface_matrix_thresholds))
        # prepare additional vars
        for row in self.interface_matrix_thresholds:
            # rospy.logwarn("======= %s====%d\n" % (str(row), len(row)))
            thresh = 0.0
            for k in xrange(self.n):
                row[k] = thresh+row[k]
                thresh = row[k]
    
    def randomize(self, intended):
        """ randomize output according to interfce matrix """
        r = random.random()
        if intended >= len(self.interface_matrix_thresholds):
            return intended
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
            self.cmd_detected.cmd = self.cmd_intended
            self.cmd_detected.cmd = self.randomize(self.cmd_intended)
            #rospy.loginfo("%s: number of connections=%d" % (rospy.get_name(), self.pub.get_num_connections()))
            self.pub.publish(self.cmd_detected)
            rospy.loginfo("%s: published detected command, intended=%d, detected=%d (SEQ==%d)", rospy.get_name(), self.cmd_intended, self.cmd_detected.cmd, self.cmd_detected.header.seq)
            if self.default_cmd == 'prev':
                pass
            elif self.default_cmd == 'rand':
                self.cmd_intended = random.randint(0,self.n)
            else:
                try:
                    self.cmd_intended = int(self.default_cmd)
                except ValueError:
                    rospy.logerr("Invalid lti model default command. Use prev, rand or any integer") 
                    exit(1)
                
    
    def cmdIntendedCallback(self, msg):
        # if self.callbackRunning:
        #     return
        # else:
        #     self.callbackRunning = True
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
    rospy.spin()