#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 21:48:47 2021

@author: tabor
"""

# thanks to https://www.asu.edu/courses/kin335/documents/Linear-Angular%20Relationships.pdf

import rospy
from std_msgs.msg import Float64, Float64
from geometry_msgs.msg import Twist

wheel_radius = 0.041
max_angular_velocity = 3.14
max_voltage = 12

def callback(twist):
    # converts linear velocity input to angular velocity output by dividing by the wheel's radius
    left = (twist.linear.x - twist.angular.z)
    right = (twist.linear.x + twist.angular.z)

    # convert angular velocity to [-1,1] and multiply max voltage
    # left = (left_vel/max_angular_velocity) * max_voltage
    # right = (right_vel/max_angular_velocity) * max_voltage
    global left_back, left_front, right_back, right_front

    left_msg = Float64()
    left_msg.data = left
    right_msg = Float64()
    right_msg.data = right
    
    left_back.publish(left_msg)
    left_front.publish(left_msg)
    right_back.publish(right_msg)
    right_front.publish(right_msg)

def left_back_callback(ang_vel):
    val = Float64()
    val.data = ang_vel.data
    left_back_pub.publish(val)

def left_front_callback(ang_vel):
    val = Float64()
    val.data = ang_vel.data
    left_front_pub.publish(val)

def right_back_callback(ang_vel):
    val = Float64()
    val.data = ang_vel.data
    right_back_pub.publish(val)

def right_front_callback(ang_vel):
    val = Float64()
    val.data = ang_vel.data
    right_front_pub.publish(val)
    
def listener():
    rospy.init_node('cmd_vel2motor', anonymous=True)

    global left_back, left_front, right_back, right_front, left_back_pub, right_front_pub, left_front_pub, right_back_pub
    left_back = rospy.Publisher('/left_back_motor/set_velocity', Float64, queue_size=10)
    left_front = rospy.Publisher('/left_front_motor/set_velocity', Float64, queue_size=10)
    right_back = rospy.Publisher('/right_back_motor/set_velocity', Float64, queue_size=10)
    right_front = rospy.Publisher('/right_front_motor/set_velocity', Float64, queue_size=10)

    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.Subscriber('/left_back_motor/velocity', Float64, left_back_callback)
    rospy.Subscriber('/left_front_motor/velocity', Float64, left_front_callback)
    rospy.Subscriber('/right_back_motor/velocity', Float64, right_back_callback)
    rospy.Subscriber('/right_front_motor/velocity', Float64, right_front_callback)
    left_front_pub = rospy.Publisher('/left_back_motor/actual_velocity', Float64, queue_size=10)
    right_front_pub = rospy.Publisher('/left_front_motor/actual_velocity', Float64, queue_size=10)
    left_back_pub = rospy.Publisher('/right_back_motor/actual_velocity', Float64, queue_size=10)
    right_back_pub = rospy.Publisher('/right_front_motor/actual_velocity', Float64, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
