#!/usr/bin/env python
import roslib
import rospy
import pygame, sys
import numpy
import time
import math
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose,Twist
from std_msgs.msg import String, Float32, ColorRGBA, Bool
from gazeboSimulation.msg import GazeboModel, GazeboModels, GazeboType, GazeboTypes, GazeboZone, GazeboZones, ChangeUpGoal, ChangeUpGoals, ChangeUpField

red = ColorRGBA()
red.r = 255
red.g = 0
red.b = 0
red.a = 1

blue = ColorRGBA()
blue.r = 0
blue.g = 0
blue.b = 255
blue.a = 1

def main():
    # publisher objects
    main.types = GazeboTypes()
    main.zones = GazeboZones()
    main.models = GazeboModels()
    main.field = ChangeUpField()

    print("logic models")
    rospy.init_node('cost_models')

    fieldPub = rospy.Publisher('/field/field', ChangeUpField, queue_size=3)

    def modelCallback(imodels):
        main.models = imodels

    def typesCallback(types):
        main.types = types

    def zonesCallback(zones):
        main.zones = zones

    def fieldCallback(field):
        main.field = field

    rospy.Subscriber('/field/models', GazeboModels, modelCallback)
    rospy.Subscriber('/field/types', GazeboTypes, typesCallback)
    rospy.Subscriber('/field/zones', GazeboZones, zonesCallback)
    rospy.Subscriber('/field/field', ChangeUpField, fieldCallback)

    def inCircle(x, y, center_x, center_y, radius):
        return (float(x) - float(center_x)) ** 2 + (float(y) - float(center_y)) ** 2 < float(radius) ** 2

    while not rospy.is_shutdown():
        try:
            time.sleep(.25)

        except rospy.ROSInterruptException:
            print("failed to display models")

if __name__ == '__main__':
    main()
