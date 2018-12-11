#!/usr/bin/env python

from geometry_msgs.msg import TransformStamped
import math
import rospy
from std_msgs.msg import Int32
import tf
import tf2_ros

import numpy as np


right_direction = 1
left_direction = 1

def callbackRight(data):
    global right_direction
    right_direction = data.data

def callbackLeft(data):
    global left_direction
    left_direction = data.data

def soundsourceLocalization(right, left):

    global right_direction
    global left_direction

    d_x = abs(right.transform.translation.x - left.transform.translation.x)
    d_y = abs(right.transform.translation.y - left.transform.translation.x)
    d = math.sqrt(d_x**2 + d_y**2)
    
    right_init_euler = tf.transformations.euler_from_quaternion((right.transform.rotation.x, right.transform.rotation.y, right.transform.rotation.z, right.transform.rotation.w))
    right_euler = math.radians(right_direction) 
    right_theta = right_euler - right_init_euler[2]

    left_init_euler = tf.transformations.euler_from_quaternion((left.transform.rotation.x, left.transform.rotation.y, left.transform.rotation.z, left.transform.rotation.w))
    left_euler = math.radians(left_direction) 
    left_theta = left_euler - left_init_euler[2]
# Temporary, need to fix
# This geometric computation is not availble when both thetas is 90 deg      
    if right_theta % 90 == 0:
        return
    if left_theta % 90 == 0:
        return

    distance_from_right = d * math.tan(left_theta)/(math.tan(right_theta)+math.tan(left_theta))

# Need error correspondence

    soundsource = right
    soundsource.transform.translation.x += distance_from_right*((-1)*math.tan(right_theta)*math.sin(right_init_euler[2])+math.cos(right_init_euler[2]))
    soundsource.transform.translation.y += distance_from_right*(math.tan(right_theta)*math.cos(right_init_euler[2])+math.sin(right_init_euler[2]))

    q = tf.transformations.quaternion_from_euler(0,0,math.pi+(right_euler+left_euler)/2)
    soundsource.transform.rotation.x = q[0]
    soundsource.transform.rotation.y = q[1]
    soundsource.transform.rotation.z = q[2]
    soundsource.transform.rotation.w = q[3]

    soundsource.header.frame_id = "base_link"
    soundsource.child_frame_id = "soundsource"

    return soundsource

if __name__ == '__main__':
    rospy.init_node('soundsource_localization')
    rate = rospy.Rate(100);
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    tfBroadcaster = tf2_ros.StaticTransformBroadcaster()
    tf_soundsource = TransformStamped()

    while not rospy.is_shutdown():    
        try:
            rightTrans = tfBuffer.lookup_transform('base_link','right_mic_link',rospy.Time())
            leftTrans = tfBuffer.lookup_transform('base_link','left_mic_link',rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        doaSub01 = rospy.Subscriber('/right_sound_direction', Int32, callbackRight)
        doaSub02 = rospy.Subscriber('/left_sound_direction', Int32, callbackLeft) 
        tf_soundsource = soundsourceLocalization(rightTrans, leftTrans)

        tfBroadcaster.sendTransform(tf_soundsource)
