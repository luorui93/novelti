#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion
import tf_conversions
        
if __name__=="__main__":
    rospy.init_node('displayed_pose_publisher')
    poses = rospy.get_param('~predefined_poses', {})
    names = rospy.get_param('~poses_to_display', ",".join(poses.keys())).split(',')
    goal = names[1]
    for name,pose in poses.iteritems():
        if name == goal:
            pub = rospy.Publisher('/displayed_pose_'+name, PoseStamped, queue_size=1, latch=True)
            p = PoseStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "/map"
            p.pose.position.x = pose['x']
            p.pose.position.y = pose['y']
            p.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, pose['yaw']))
            pub.publish(p)
    rospy.spin()