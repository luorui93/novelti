#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from novelti.msg import Command
from rosgraph_msgs.msg import Log
from geometry_msgs.msg import PoseStamped

#sudo apt-get install ros-kinetic-sound-play
# or clone sources and do:  sudo apt-get install festival



class AudioStatusReporter:
    def __init__(self):
        self.mp3dir = rospy.get_param('~mp3dir', None)
        self.mode = rospy.get_param('~mode', "emotiv")
        self.sub_pose_arrived  = rospy.Subscriber('/pose_arrived',  PoseStamped, self.poseArrivedCb)
        self.sub3 = rospy.Subscriber('/rosout', Log, self.rosoutCb)
        self.phrases = {
            "just_started_emotiv"           : "Make first expression several times to start navigation",
            "just_started_button"           : "Push button for several times to start navigation",
            "position_control_started"      : "Position control started" ,
            "orientation_control_started"   : "Position inferred. Starting orientation control",
            "orientation_inferred"          : "Destination fully inferred",
            "can_reset_emotiv"              : "",
            "can_reset_button"              : "",
            "arrived_emotiv"                : "You arrived to your destination. Make first expression several times to start navigation.",
            "arrived_button"                : "You arrived to your destination. Push button for several times to start navigation.",
            "test"                          : "test",
        }
        self.state = "JUST_STARTED"
        self.soundhandle = SoundClient()
        
    def run(self):
        rospy.spin()
    
    def say(self, event):
        if self.mp3dir:
            self.soundhandle.playWave(self.mp3dir + "/" + event + ".mp3", 0.8)
        else:
            self.soundhandle.say(self.phrases[event]) #)
    
    def rosoutCb(self, msg):
        if   msg.msg.startswith("/novelti_shared_control: >>>>>>>>>>>>>>>>>> Starting new inference: Action Control"):
            if self.state=="JUST_STARTED":
                self.say("just_started_"+self.mode)
            else:
                #self.say("can_reset_"+self.mode)
                pass
        elif msg.msg.startswith("/novelti_shared_control: >>>>>>>>>>>>>>>>>> Starting new inference: Position Control"):
            self.state = "POSITION_INFERENCE"
            self.say("position_control_started")
        elif msg.msg.startswith("/novelti_shared_control: >>>>>>>>>>>>>>>>>> Starting new inference: Orientation Control"):
            self.state = "ORIENTATION_INFERENCE"
            self.say("orientation_control_started")
        elif msg.msg.startswith("/novelti_shared_control: ORIENTATION INFERRED"):
            self.state = "DRIVING_TO_INFERRED"
            self.say("orientation_inferred")

    def poseArrivedCb(self, msg):
        if self.state=="DRIVING_TO_INFERRED":
            self.state = "ARRIVED"
            self.say("arrived_"+self.mode)

if __name__=="__main__":
    rospy.init_node('audio_status_reporter')
    asr = AudioStatusReporter()
    asr.run()

