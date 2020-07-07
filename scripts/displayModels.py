#!/usr/bin/env python
import roslib
import rospy
import pygame, sys
import numpy
import time
import math
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from gazeboSimulation.msg import GazeboModel, GazeboModels, GazeboType, GazeboTypes, GazeboZone, GazeboZones
from tf.transformations import euler_from_quaternion, quaternion_from_euler

fieldPixels = 1000
fieldConvM = (fieldPixels / 140.5) * 39.3701
size = width, height = fieldPixels, fieldPixels

def meterToPixels(meter):
    out = meter * fieldConvM
    return int(out)

def pixelsToMeter(pixels):
    out = pixels / (39.3701 * fieldConvM)
#    print("pixels to meter: " + str(pixels) + ", " + str(out))
    return float(out)

def xyToOriginPoint(x, y):
    ix = x + fieldPixels / 2
    iy = -y + fieldPixels / 2
    return (int(ix), int(iy))

def xToOriginPoint(x):
    ix = x + fieldPixels / 2
    return int(ix)

def yToOriginPoint(y):
    iy = -y + fieldPixels / 2
    return int(iy)

def inCircle(x, y, center_x, center_y):
    return (x - center_x) ** 2 + (y - center_y) ** 2 < robotRadius ** 2

def draw(screen, shape, color, x1, y1, x2, y2, radius, width):
    if(shape == 'circle'):
        point = (x1, y1)
        pygame.draw.circle(screen, color, point, radius, width)
    elif(shape == 'rect'):
        size = (x2 - x1, y2 - y1)
        rect = pygame.Rect(xyToOriginPoint(x1, -y1), size)
        pygame.draw.rect(screen, color, rect, width)

def main():
    pygame.init()
    pygame.display.set_caption('Field')
    screen = pygame.display.set_mode(size)

    print("display models")
    rospy.init_node('display_models')

    # publisher objects
    main.types = GazeboTypes()
    main.zones = GazeboZones()

    def zonesCallback(zones):
#        print("zonesCallback")
        main.zones = zones

    rospy.Subscriber("/field/zones", GazeboZones, zonesCallback)

    def typesCallback(types):
#        print("typesCallback")
        main.types = types

    rospy.Subscriber("/field/types", GazeboTypes, typesCallback)

    while not rospy.is_shutdown():
        try:
            time.sleep(.25)

            for zone in main.zones.zones:
                zoneType = GazeboType()
                for type in main.types.types:
                    if(type.name == zone.type):
                        zoneType = type
                draw(screen,
                     zoneType.shape,
                     (zone.color.r,zone.color.g,zone.color.b),
                     xToOriginPoint(meterToPixels(float(zone.x1.data))),
                     yToOriginPoint(meterToPixels(float(zone.y1.data))),
                     xToOriginPoint(meterToPixels(float(zone.x2.data))),
                     yToOriginPoint(meterToPixels(float(zone.y2.data))),
                     meterToPixels(float(zoneType.radius.data)),
                     meterToPixels(float(zoneType.width.data)))

                for model in zone.models.models:
                    modelType = GazeboType()
                    for type in main.types.types:
                        if (type.name == model.type):
                            modelType = type
                    if(model.zone == zone.name):
#                        print(model.name + ": in: " + zone.name)
#                        print(xToOriginPoint(meterToPixels(float(model.pose.position.x))),
#                             yToOriginPoint(meterToPixels(float(model.pose.position.y))))
                        # WARNING: ONLY CIRCLES WORK FOR THIS AT THE MOMENT
                        draw(screen,
                             modelType.shape,
                             (model.color.r, model.color.g, model.color.b),
                             xToOriginPoint(meterToPixels(float(model.pose.position.x))),
                             yToOriginPoint(meterToPixels(float(model.pose.position.y))),
                             float(0),
                             float(0),
                             meterToPixels(float(modelType.radius.data)),
                             int(modelType.width.data))

            pygame.display.flip()

        except rospy.ROSInterruptException:
            print("failed to display models")

if __name__ == '__main__':
    main()
