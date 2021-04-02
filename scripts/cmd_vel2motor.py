#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 21:48:47 2021

@author: tabor
"""

import rospy
from std_msgs.msg import Float64, Float64
from geometry_msgs.msg import Twist

def callback(twist):
    left = twist.linear.x/1.5 - twist.angular.z/3.0
    right = twist.linear.x/1.5 + twist.angular.z/3.0
    global left_back, left_front, right_back, right_front 

    left_msg = Float64()
    left_msg.data = left*12.0
    
    right_msg = Float64()
    right_msg.data = right*12.0
    
    left_back.publish(left_msg)
    left_front.publish(left_msg)

    right_back.publish(right_msg)
    right_front.publish(right_msg)

    
def listener():
    rospy.init_node('cmd_vel2motor', anonymous=True)

    
    global left_back, left_front, right_back, right_front 
    left_back = rospy.Publisher('/left_back_motor/command', Float64, queue_size=10)
    left_front = rospy.Publisher('/left_front_motor/command', Float64, queue_size=10)
    right_back = rospy.Publisher('/right_back_motor/command', Float64, queue_size=10)
    right_front = rospy.Publisher('/right_front_motor/command', Float64, queue_size=10)

    
    rospy.Subscriber("/cmd_vel", Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()