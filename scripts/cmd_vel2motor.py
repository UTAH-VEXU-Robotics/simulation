#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 21:48:47 2021

@author: tabor
"""

# thanks to https://www.asu.edu/courses/kin335/documents/Linear-Angular%20Relationships.pdf

import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist

wheel_radius = 0.041
max_angular_velocity = 3.14
max_voltage = 14


def callback(twist):
    # converts linear velocity input to angular velocity output by dividing by the wheel's radius
    left = (twist.linear.x - twist.angular.z)
    right = (twist.linear.x + twist.angular.z)

    global pid_left_set, pid_right_set, mode
    left_msg = Float64();
    left_msg.data = left
    right_msg = Float64();
    right_msg.data = right

    pid_left_set.publish(left_msg)
    pid_right_set.publish(right_msg)

def left_callback(vel):
    global back_left_command, front_left_command
    set = Float64();
    set.data = vel.data
    back_left_command.publish(set)
    front_left_command.publish(set)

def right_callback(vel):
    global back_right_command, front_right_command
    set = Float64();
    set.data = vel.data
    back_right_command.publish(set)
    front_right_command.publish(set)

def back_left_callback(vel):
    global back_left_velocity
    back_left_velocity = vel.data

def front_left_callback(vel):
    global front_left_velocity
    front_left_velocity = vel.data

def back_right_callback(vel):
    global back_right_velocity
    back_right_velocity = vel.data


def front_right_callback(vel):
    global front_right_velocity
    front_right_velocity = vel.data


def listener():
    rospy.init_node('cmd_vel2motor', anonymous=True)

    global \
        pid_left_set, \
        pid_right_set, \
        pid_left_state, \
        pid_right_state, \
        back_left_command, \
        front_left_command, \
        back_right_command, \
        front_right_command, \
        back_left_velocity, \
        front_left_velocity, \
        back_right_velocity, \
        front_right_velocity, \
        mode

    pid_left_set = rospy.Publisher('/pid_motor_left/setpoint', Float64, queue_size=10)
    pid_right_set = rospy.Publisher('/pid_motor_right/setpoint', Float64, queue_size=10)
    pid_left_state = rospy.Publisher('/pid_motor_left/state', Float64, queue_size=10)
    pid_right_state = rospy.Publisher('/pid_motor_right/state', Float64, queue_size=10)
    back_left_command = rospy.Publisher('/motor_back_left/command', Float64, queue_size=10)
    front_left_command = rospy.Publisher('/motor_front_left/command', Float64, queue_size=10)
    back_right_command = rospy.Publisher('/motor_back_right/command', Float64, queue_size=10)
    front_right_command = rospy.Publisher('/motor_front_right/command', Float64, queue_size=10)
    pid_enable = rospy.Publisher('/pid_enable', Bool, queue_size=10)

    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.Subscriber("/pid_motor_left/control_effort", Float64, left_callback)
    rospy.Subscriber("/pid_motor_right/control_effort", Float64, right_callback)
    rospy.Subscriber("/motor_back_left/velocity", Float64, front_left_callback)
    rospy.Subscriber("/motor_front_left/velocity", Float64, front_right_callback)
    rospy.Subscriber("/motor_back_right/velocity", Float64, back_left_callback)
    rospy.Subscriber("/motor_front_right/velocity", Float64, back_right_callback)

    back_left_velocity = 0
    front_left_velocity = 0
    back_right_velocity = 0
    front_right_velocity = 0

    rate = rospy.Rate(10)  # in hz
    while not rospy.is_shutdown():
        rate.sleep()
        # enable = Bool()
        # enable.data = True
        # pid_enable.publish(enable)
        left_velocity = Float64()
        left_velocity.data = (back_left_velocity * front_left_velocity / 2)
        pid_left_state.publish(left_velocity)
        rate.sleep()
        right_velocity = Float64()
        right_velocity.data = (back_right_velocity * front_right_velocity / 2)
        pid_right_state.publish(right_velocity)

if __name__ == '__main__':
    listener()
