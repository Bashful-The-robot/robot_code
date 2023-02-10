#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math

rospy.init_node('transform_map_odom')
br = tf2_ros.TransformBroadcaster()

t = TransformStamped()
t.header.frame_id = "odom"
t.child_frame_id = "camera_link"


tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        trans = tfBuffer.lookup_transform("odom", "base_link", rospy.Time.now(), rospy.Duration(1.0))
        t.header.stamp = trans.header.stamp

        t.transform.rotation.x = trans.transform.rotation.x 
        t.transform.rotation.y = trans.transform.rotation.y 
        t.transform.rotation.z = trans.transform.rotation.z 
        t.transform.rotation.w = trans.transform.rotation.w 

        t.transform.translation.x = trans.transform.translation.x 
        t.transform.translation.y = trans.transform.translation.y
        t.transform.translation.z = trans.transform.translation.z + 0.05
        print(t)
        br.sendTransform(t)
        rate.sleep()
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue

if __name__ == '__main__':
    rospy.spin()