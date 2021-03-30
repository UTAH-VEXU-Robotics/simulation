#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 21:48:47 2021

@author: tabor
"""

import rospy
from std_msgs.msg import Int32
import numpy as np
import matplotlib.pyplot as plt


def left_back_callback(enc):

    global left_back, left_front, right_back, right_front 
    left_back = enc.data
def left_front_callback(enc):

    global left_back, left_front, right_back, right_front 
    left_front = enc.data
def right_back_callback(enc):

    global left_back, left_front, right_back, right_front 
    right_back = enc.data
def right_front_callback(enc):

    global left_back, left_front, right_back, right_front 
    right_front = enc.data

class odomv1:
    def __init__(self,chassis_width,ticks_per_rotation,wheel_diameter):
        self.w = chassis_width
        self.last_left_back = 0
        self.last_left_front = 0
        self.last_right_back = 0
        self.last_right_front =0
        
        self.tpr = ticks_per_rotation
        self.dia = wheel_diameter
        self.x = 0 
        self.y = 0
        self.theta = 0
    def update(self,left_back, left_front, right_back, right_front):
        delta_left = ((left_back - self.last_left_back) + (left_front - self.last_left_front))/2
        delta_right = ((right_back - self.last_right_back) + (right_front - self.last_right_front))/2
        
        leftDist = delta_left*np.pi*self.dia/self.tpr
        rightDist = delta_right*np.pi*self.dia/self.tpr
        
        straightDist = (leftDist + rightDist) / 2
        outsideDist = (leftDist - rightDist) / 2
        
        chassis_circumference = (self.w * np.pi)
        deltaAngle = 2*np.pi * outsideDist/chassis_circumference
        
        self.x += np.cos(self.theta) *straightDist
        self.y += np.sin(self.theta) *straightDist
        self.theta += deltaAngle
        
        
        self.last_left_back = left_back
        self.last_left_front = left_front
        self.last_right_back = right_back
        self.last_right_front =right_front
        
        
        return (self.x,self.y,self.theta)


        

def listener():
    rospy.init_node('odom', anonymous=True)

    
    global left_back, left_front, right_back, right_front 
    left_back_sub = rospy.Subscriber('/left_back_motor/encoder', Int32, left_back_callback)
    left_front_sub = rospy.Subscriber('/left_front_motor/encoder', Int32, left_front_callback)
    right_back_sub = rospy.Subscriber('/right_back_motor/encoder', Int32, right_back_callback)
    right_front_sub = rospy.Subscriber('/right_front_motor/encoder', Int32, right_front_callback)
    left_back = left_front = right_back = right_front = 0
    rate = rospy.Rate(100) # 10hz
    v1 = odomv1(0.2/1.018, 900, 0.1126)
    
    v1poses = []
    i = 0
    fig = plt.figure()
    while not rospy.is_shutdown():
        i +=1
        v1poses.append(v1.update(left_back, left_front, right_back, right_front ))
        rate.sleep()
        if i % 200 == 0:
            npv1poses = np.array(v1poses)
            print(npv1poses[-1,:])
            plt.plot(npv1poses[:,0],npv1poses[:,1])
            plt.show()

if __name__ == '__main__':

    listener()