#!/usr/bin/env python

from geometry_msgs.msg import TransformStamped
import math
import rospy
from std_msgs.msg import Int32
import tf
import tf2_ros

right_direction = 0
left_direction = 0

def callbackRight(data):
    global right_direction
    right_direction = data.data * (-1) - 90

def callbackLeft(data):
    global left_direction
    left_direction = data.data * (-1) - 90

def soundsourceLocalization(right, left):

    global right_direction
    global left_direction

    l_x = abs(right.transform.translation.x - left.transform.translation.x)
    l_y = abs(right.transform.translation.y - left.transform.translation.y)
    l = math.sqrt(l_x**2 + l_y**2)
    
    right_init_euler = tf.transformations.euler_from_quaternion((right.transform.rotation.x, right.transform.rotation.y, right.transform.rotation.z, right.transform.rotation.w))
    right_rad = math.radians(right_direction) 
    right_euler = right_rad + right_init_euler[2]

    left_init_euler = tf.transformations.euler_from_quaternion((left.transform.rotation.x, left.transform.rotation.y, left.transform.rotation.z, left.transform.rotation.w))
    left_rad = math.radians(left_direction) 
    left_euler = left_rad + left_init_euler[2]
# Temporary, need to fix
# This geometric computation is not availble when both thetas are 90 deg      
    if not (left_euler - right_euler) % math.pi == 0 \
    and not (left_euler - math.pi/2) % math.pi == 0 \
    and not (right_euler - math.pi/2) % math.pi == 0 :
        if not 5*math.sin(left_euler - right_euler) - l*math.sin(right_euler) >= 0 \
        and not 5*math.sin(left_euler - right_euler) - l*math.sin(left_euler) >= 0 :
            d = ((math.sin(right_euler)*math.sin(left_euler))/((-1)*math.sin(left_euler-right_euler)))*l
            lx_from_left = d/math.tan(left_euler)
        else:
            check_right = 5*math.sin(left_euler - right_euler) - l*math.sin(left_euler)
            check_left = 5*math.sin(left_euler - right_euler) - l*math.sin(right_euler)
            rospy.logwarn("Over the Range of Microphones can decect %f %f " %(check_right, check_left ))
            return
    else:
        rospy.logwarn("(%f , %f)" %(math.degrees(right_euler), math.degrees(left_euler)))
        return

# Need error correspondence

    soundsource = left
    soundsource.transform.translation.x += d
    soundsource.transform.translation.y -= lx_from_left

    rospy.loginfo("Soundsource (%f , %f)" % (soundsource.transform.translation.x, soundsource.transform.translation.y))

    q = tf.transformations.quaternion_from_euler(0,0,math.pi+(right_rad+left_rad)/2)
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
    tfBroadcaster = tf2_ros.TransformBroadcaster()
    tf_soundsource = TransformStamped()
    buff_doa_right = 0
    buff_doa_left = 0

    while not rospy.is_shutdown():    
        try:
            rightTrans = tfBuffer.lookup_transform('base_link','right_mic_link',rospy.Time())
            leftTrans = tfBuffer.lookup_transform('base_link','left_mic_link',rospy.Time())
            doaSub_right = rospy.Subscriber('/right_sound_direction', Int32, callbackRight)
            doaSub_left = rospy.Subscriber('/left_sound_direction', Int32, callbackLeft) 
            if not right_direction == buff_doa_right or not left_direction == buff_doa_left: 
                tf_soundsource = soundsourceLocalization(rightTrans, leftTrans)
                if not tf_soundsource == None:
                    tfBroadcaster.sendTransform(tf_soundsource)
                rospy.loginfo("Right Direction : %d , Left Direction : %d  " %(right_direction,left_direction))
                buff_doa_right = right_direction
                buff_doa_left = left_direction
                rate.sleep()
            else:
                rate.sleep()
                continue
        except (rospy.ROSInterruptException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
