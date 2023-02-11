#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math

global x,y,yaw
x=0
y=0
yaw=0

def encoder_callback(msg):
    global x,y,yaw
    rospy.loginfo('New encoder received:\n%s', msg)

    br = tf2_ros.TransformBroadcaster()

    t = TransformStamped()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.header.stamp = msg.header.stamp
    
    # TODO: Fill in
    ticks_per_rev = 3072
    r = 0.04921
    B = 0.3
    rate = 20 #50ms, 20Hz
    K = 0.002
    E_r = msg.delta_encoder_right
    E_l = msg.delta_encoder_left
    T_r = msg.delta_time_right
    T_l = msg.delta_time_left

    D = (r/2)*(K*E_r + K*E_l)
    delta_theta = (r/B)*(K*E_r - K*E_l)

    x = x + D*math.cos(yaw)
    y = y + D*math.sin(yaw)
    yaw = yaw + delta_theta

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0
    
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    print(t)

    br.sendTransform(t)

rospy.init_node('odometry')
sub_goal = rospy.Subscriber('/motor/encoders', Encoders, encoder_callback)

if __name__ == '__main__':
    rospy.spin()
