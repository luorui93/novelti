#!/usr/bin/env python

import re
import math
from threading import Thread

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import *
from low_throughput_hmi.msg import IntMap
from low_throughput_hmi.msg import Experiment


STATE = "init"

def wait_for_all(dependent_nodes, wait_for_srvs, is_test):
    stop_services={}
    for n in dependent_nodes:
        rospy.wait_for_service('/'+n+'/stop')
        stop_services[n] = rospy.ServiceProxy('/'+n+'/stop', Empty)
    for srv in wait_for_srvs:
        rospy.wait_for_service(srv) #wait for rviz
    return stop_services

def xy_to_pose(xy, resolution):
    p = Pose()
    p.position.x = (xy[0]+0.5)*resolution
    p.position.y = (xy[1]+0.5)*resolution
    return p
    
def publish_map():
    global map_prefix, EXP, pose_iter, pub_map
    (width, height, data) = read_map_from_file(map_prefix + EXP['map'])
    pose_iter = iter(EXP['poses'])
    msg = Experiment()
    msg.header.stamp = rospy.Time.now()
    msg.name = EXP['name']
    msg.mapa.header.frame_id = "/map"
    msg.mapa.data = data
    msg.mapa.info.width = width
    msg.mapa.info.height = height
    msg.mapa.info.resolution = EXP['resolution']
    msg.init_pose = xy_to_pose(pose_iter.next(), EXP['resolution'])
    pub_map.publish(msg)
    rospy.loginfo("experimentator: ========================================= published %dx%d scene (resolution=%f), experiment name=%s ==============================================", msg.mapa.info.width, msg.mapa.info.height, msg.mapa.info.resolution, msg.name)

def stop_all():
    global stop_services
    for key,val in stop_services.iteritems():
        val()
    
def publish_next_intended_pose():
    global pose_iter, NEXT_DEST
    #save data
    NEXT_DEST = next(pose_iter, None)
    if NEXT_DEST is not None:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose = xy_to_pose(NEXT_DEST, EXP['resolution'])
        pose.header.frame_id="/map"
        pub_intended_pose.publish(pose)
        rospy.loginfo("experimentator: ============= published new intended pose, xy=(%d,%d), pose=(%f,%f) =============", NEXT_DEST[0], NEXT_DEST[1], pose.pose.position.x, pose.pose.position.y)
    else:
        stop_all()
        publish_next_attempt()
    
def publish_next_attempt():
    global exp_iter, EXP, ITERATIONS
    
    if (ITERATIONS==0):
        EXP = next(exp_iter, None)
        if EXP is None:
            rospy.signal_shutdown("Finihed all experiments")
            return
        ITERATIONS = EXP['tries']
    rospy.sleep(1)
    publish_map()
    rospy.sleep(1)
    publish_next_intended_pose()
    ITERATIONS-=1

    
def read_map_from_file(fname):
    height = None
    width  = None
    data = None
    with open(fname) as f:
        for k in range(0,4):
            line = f.readline()
            m = re.match(r"\s*height\s+(\d+)\s*", line)
            if m:
                height = int(m.group(1))
            m = re.match(r"\s*width\s+(\d+)\s*", line)
            if m:
                width = int(m.group(1))
        if width is None or height is None:
            raise Exception("Failed to parse map file: couldn't read width or height")
        else:
            rospy.loginfo("experimentator: read map meta data from file '%s', width=%d, height=%d", fname, width, height)
            data = [0]*(width*height)

            for y in range(height-1,-1,-1):
                line = f.readline().rstrip()
                #rospy.loginfo("experimentator: map line (y=%d, length=%d) read= %s", y, len(line), y*width, (y+1)*width-1, str([(-1 if c=="." else -2) for c in line]))
                data[y*width:(y+1)*width] = [(-1 if c=="." else -10) for c in line]
    return (width, height, data)

def form_experiment_msg(data, width, height, resolution):
    msg = IntMap()
    msg.header.frame_id = "/map"
    msg.data = data
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    return msg

def test_timer(period):
    while(not rospy.is_shutdown()):
        rospy.sleep(period)
        publish_next_intended_pose()
        
def identifiedPoseCallback(msg):
    global STATE
    STATE = "wait_for_arrival"
    rospy.loginfo("experimentator: swithched to '%s' STATE", STATE)

def pose2xy(pose, resolution):
    return (
        int(math.floor(pose.pose.position.x/resolution)),
        int(math.floor(pose.pose.position.y/resolution))
    )
    
def currentPoseCallback(msg):
    global STATE, EXP, NEXT_DEST
    if STATE == "wait_for_arrival":
        xy = pose2xy(msg, EXP['resolution'])
        if xy[0]==NEXT_DEST[0] and xy[1]==NEXT_DEST[1]:
            STATE = "arrived"
            rospy.loginfo("experimentator: swithched to '%s' STATE", STATE)
            rospy.sleep(1)
            publish_next_intended_pose()
    
    
if __name__=="__main__":
    global exp_iter, pub_intended_pose, pub_map, map_prefix, stop_services, ITERATIONS
    rospy.init_node('experimentator')
    map_prefix = rospy.get_param('~map_prefix', 'default_map_prefix')
    is_test = rospy.get_param('~is_test', False)
    dependent_nodes = rospy.get_param('~dependent_nodes', [])
    wait_for_srvs   = rospy.get_param('~wait_for_srvs', [])
    experiment_set = rospy.get_param('~experiment_set', {})
    exp_name = rospy.get_param('~experiment', "")
    tries = 0
    if exp_name=="":
        exp_names = rospy.get_param('~experiments', [])
    else:
        tries = rospy.get_param('~tries', 0)
        exp_names = [exp_name]
    exps = []
    for n in exp_names:
        experiment_set[n]['name'] = n
        if tries != 0:
             experiment_set[n]['tries'] = tries
        exps.append(experiment_set[n])
    rospy.loginfo("experimentator: going to run the following experiments: %s", str(exps))
    exp_iter = iter(exps)
    
    pub_map = rospy.Publisher('/scene', Experiment)
    pub_intended_pose = rospy.Publisher('/intended_pose', PoseStamped, latch=False)
    sub_ident_pose = rospy.Subscriber('/identified_pose', PoseStamped, identifiedPoseCallback)
    current_pose   = rospy.Subscriber('/current_pose', PoseStamped, currentPoseCallback)

    stop_services = wait_for_all(dependent_nodes, wait_for_srvs, is_test)
    
    rospy.sleep(1)
    if is_test:
        Thread(target=test_timer, args=[7]).start()
    
    ITERATIONS=0
    publish_next_attempt()
    
    rospy.spin()
    
    #pub.publish()