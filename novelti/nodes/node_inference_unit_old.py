#!/usr/bin/env python

import rospy, tf
from math import pi

from novelti.msg import Command
from novelti.msg import FloatMap
from geometry_msgs.msg import PoseStamped
from novelti.msg import IntMap
from novelti.msg import Experiment
from std_srvs.srv import *

divided_map = None
priors = None
pdf = None


def xy_to_pose(xy, resolution):
    p = PoseStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "/map"
    p.pose.position.x = (xy[0]+0.5)*resolution
    p.pose.position.y = (xy[1]+0.5)*resolution
    return p

def sceneCallback(msg):
    global pub_pdf, pdf, STATE, sub_div_map, sub_det_cmd, nav_task_sub, uni_prob
    rospy.loginfo("inference_module: got scene, restarting")
    
    STATE = "LOCALIZING"
    
    #init pdf
    total_pixels = sum(1 for p in msg.mapa.data if p>=-1)
    uni_prob = 1.0/total_pixels
    pdf = FloatMap()
    pdf.header.frame_id = "/map"
    pdf.info.width = msg.mapa.info.width
    pdf.info.height = msg.mapa.info.height
    pdf.info.resolution = msg.mapa.info.resolution
    pdf.data = [ (uni_prob if p>=-1 else -10.0) for p in msg.mapa.data ]
    
    sub_div_map = rospy.Subscriber('/divided_map', IntMap, dividedMapCallback)
    sub_det_cmd = rospy.Subscriber('/detected_command', Command, userCmdCallback)    
    nav_task_sub = rospy.Subscriber('/intended_pose', PoseStamped, navTaskCallback) #serves as a signal for new navigation task

def dividedMapCallback(msg):
    rospy.loginfo("inference_module: received divided_map")
    global pdf, divided_map, priors, interface_matrix, STATE
    if STATE=="LOCALIZED":
        return
    divided_map = msg
    n_opts = len(interface_matrix)
    priors = [0.0]*n_opts
    for idx, p in enumerate(divided_map.data):
        if p>=0:
            priors[p]+=pdf.data[idx]
    rospy.loginfo("inference_module: priors from divided_map   : %s" % str(priors))

def calcPosteriors(detected):
    global interface_matrix, priors
    posteriors = [0.0]*len(interface_matrix)
    for k in range(0,len(interface_matrix)):
        posteriors[k] += interface_matrix[k][detected] * priors[k] #TODO double check
        #rospy.loginfo("inference_module: priors = %s" % str(priors))
        #rospy.loginfo("inference_module: interface_matrix[k][detected] = %s, k=%d, detected=%d", str(interface_matrix[k][detected]), k, detected)
    #rospy.loginfo("inference_module: posteriors=%s", str(posteriors))
    total = sum(posteriors)
    posteriors = [p/total for p in posteriors]
    return posteriors

def publishIdentifiedPose(idx):
    global divided_map, pub_ident_pose
    y = idx / divided_map.info.width
    x = idx % divided_map.info.width
    p = xy_to_pose((x,y),divided_map.info.resolution)
    pub_ident_pose.publish(p)
    rospy.loginfo("inference_module: published /identified_pose. xy=(%d,%d), pose=(%f,%f)", x,y, p.pose.position.x, p.pose.position.y)

def normalizePdf():
    global pdf
    #rospy.loginfo("inference_module: normalizing pdf, len(data)=%d", len(pdf.data))
    eps = 10**-12
    s = sum([eps-p for p in pdf.data if (p>=0.0 and p<eps)])
    pdf.data = [p if p<0.0 else (p*(1-s) if p>=eps else eps) for p in pdf.data] #(p*(1-s) if p<eps else eps)
    #rospy.loginfo("inference_module: normalized pdf sum=%f, len(data)=%d", s, len(pdf.data))

    
def userCmdCallback(msg):
    global pdf, STATE, priors, view_tf
    if STATE=="LOCALIZED":
        STATE = "NEW_LOCATION"
        rospy.loginfo("inference_module: switched to NEW_LOCATION state")
    rospy.loginfo("inference_module: ====== NEW USER COMMAND RECEIVED = %d ======, STATE=%s", msg.cmd, STATE)
    #rospy.loginfo("inference_module: pdf before update    . resolution=%f, pdf()=%f" % (pdf.info.resolution, pdf.data[346]))
    posteriors = calcPosteriors(msg.cmd)
    rospy.loginfo("inference_module: priors    : %s" % str(priors))
    rospy.loginfo("inference_module: posteriors: %s" % str(posteriors))
    coeffs = [(posteriors[k]/priors[k] if priors[k]!=0.0 else 0.0) for k in range(0, len(priors))]
    max_prob = 0.0
    pre_max_prob = 0.0
    max_prob_idx = 0
    viewable_prob = uni_prob*0.1
    limits=[pdf.info.width,0,pdf.info.height,0]
    for idx, p in enumerate(pdf.data):
        #rospy.loginfo("inference_module: idx=%d, len(coeffs)=%d, "%(idx, len(priors)))
        if p>=0:
            q = divided_map.data[idx]
            pdf.data[idx]=p*coeffs[divided_map.data[idx]]
            if pdf.data[idx] >= max_prob:
                pre_max_prob = max_prob
                max_prob = pdf.data[idx]
                max_prob_idx = idx
    for idx, p in enumerate(pdf.data):
        if p>=0:
            if pdf.data[idx] >= max_prob*0.01:
                y = idx / pdf.info.width
                x = idx % pdf.info.width
                if x<limits[0]:
                    limits[0] = x
                if x>limits[1]:
                    limits[1] = x
                if y<limits[2]:
                    limits[2] = y
                if y>limits[3]:
                    limits[3] = y 
    if STATE=="LOCALIZING" and max_prob>=0.999:
        rospy.loginfo("inference_module: LOCALIZED DESTINATION, NOT PUBLISHING PDF, max_prob=%f", max_prob);
        publishIdentifiedPose(max_prob_idx)
        STATE = "LOCALIZED"
        return
    if STATE=="NEW_LOCATION" and max_prob<0.5:
        STATE="LOCALIZING"
        rospy.loginfo("inference_module: switched to LOCALIZING state")
    normalizePdf()
    pdf.header.stamp = rospy.Time.now()
    #pdf.info.origin.position.z = 0.5
    pub_pdf.publish(pdf)
    rospy.loginfo("inference_module: published updated pdf. resolution=%f, max_prob=%f, pre_max_prob=%f, STATE=%s ", 
        pdf.info.resolution, max_prob, pre_max_prob, STATE)
    
    w = pdf.info.resolution*pdf.info.width
    h = pdf.info.resolution*pdf.info.height
    #limits=(w/2, w, h/2, h)
    #c = ((limits[0]+limits[1])*pdf.info.resolution/2, (limits[2]+limits[3])*pdf.info.resolution/2)
    max_area_size = pdf.info.resolution*max(limits[1]-limits[0], limits[3]-limits[2])
    cam_distance = 2+2*(int(max_area_size*0.2)/2)
    cam_x = (limits[0]+limits[1])*pdf.info.resolution/2
    cam_y = (limits[2]+limits[3])*pdf.info.resolution/2
    if cam_distance==8:
        cam_x = w/2
        cam_y = h/2
    view_tf.sendTransform(  (cam_x, cam_y, cam_distance),
                     tf.transformations.quaternion_from_euler(0.0, pi/2, pi/2),
                     rospy.Time.now(), "/autoview", "/map")
        

def navTaskCallback(msg):
    global pub_pdf, pdf
    if pdf is not None:
        pub_pdf.publish(pdf)
        rospy.loginfo("inference_module: got new navigation task, published pdf")
    
def stop(req):
    global sub_div_map, sub_det_cmd
    sub_div_map.unregister()
    sub_det_cmd.unregister()
    nav_task_sub.unregister()
    rospy.loginfo("inference_module: stopped")
    return EmptyResponse()

    
if __name__=="__main__":
    global pub_pdf, interface_matrix, pub_ident_pose, view_tf
    
    rospy.init_node('inference_module')
    interface_matrix = rospy.get_param('~interface_matrix', [])
    sub1    = rospy.Subscriber('/scene', Experiment, sceneCallback)
    pub_pdf = rospy.Publisher('/pdf', FloatMap, latch=False)
    pub_ident_pose = rospy.Publisher('/identified_pose', PoseStamped, latch=False)
    view_tf = tf.TransformBroadcaster()
    s = rospy.Service('~stop', Empty, stop)
    rospy.spin()
