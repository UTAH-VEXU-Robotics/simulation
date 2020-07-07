#!/usr/bin/env python
import roslib
import rospy
import pygame, sys
import numpy
import time
import math
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose,Twist
from std_msgs.msg import String, Float32, ColorRGBA
from gazeboSimulation.msg import GazeboModel, GazeboModels, GazeboType, GazeboTypes, GazeboZone, GazeboZones

black = ColorRGBA()
black.r = 0
black.g = 0
black.b = 0
black.a = 1

grey = ColorRGBA()
grey.r = 100
grey.g = 100
grey.b = 100
grey.a = 1

white = ColorRGBA()
white.r = 255
white.g = 255
white.b = 255
white.a = 1

red = ColorRGBA()
red.r = 255
red.g = 0
red.b = 0
red.a = 1

green = ColorRGBA()
green.r = 0
green.g = 255
green.b = 0
green.a = 1

blue = ColorRGBA()
blue.r = 0
blue.g = 0
blue.b = 255
blue.a = 1

ballRadius = Float32()
ballRadius.data = 0.08001
ballWidth = Float32()
ballWidth.data = 0

robotRadius = Float32()
robotRadius.data = 0.3
robotWidth = Float32()
robotWidth.data = 1

goalRadius = Float32()
goalRadius.data = 0.130048
goalWidth = Float32()
goalWidth.data = 0.054229

fieldRadius = Float32()
fieldRadius.data = 0
fieldWidth = Float32()
fieldWidth.data = 1

lft = Float32()
lft.data = -1.6335
mid = Float32()
mid.data = 0
rgt = Float32()
rgt.data = 1.6335

top = Float32()
top.data = 1.6335
dwn = Float32()
dwn.data = -1.6335

fieldX1 = Float32()
fieldX1.data = 0
fieldY1 = Float32()
fieldY1.data = 0
fieldX2 = Float32()
fieldX2.data = -1.82*2
fieldY2 = Float32()
fieldY2.data = 1.82*2

zip = Float32()
zip.data = 0

def main():
    # publisher objects
    main.types = GazeboTypes()
    main.zones = GazeboZones()
    main.models = GazeboModels()

    # the initial vars to just change later
    initialPose = Pose()
    initialModels = GazeboModels()

    itypes = [
        ['robot','circle',robotRadius,robotWidth],
        ['ball', 'circle',ballRadius,ballWidth],
        ['goal', 'circle',goalRadius,goalWidth],
        ['field', 'rect', zip, zip],
    ]

    for itype in itypes:
        type = GazeboType()
        type.name = itype[0]
        type.shape = itype[1]
        type.radius = itype[2]
        type.width = itype[3]
        main.types.types.append(type)

    izones = [
        # name           #types     #x1      #y1      #x2   #y2    #radius
        ['field', 'field', grey, fieldX1, fieldY1, fieldX2, fieldY2, initialModels],
        ['lft_top_goal', 'goal', black, lft, top, zip, zip, initialModels],
        ['mid_top_goal', 'goal', black, mid, top, zip, zip, initialModels],
        ['rgt_top_goal', 'goal', black, rgt, top, zip, zip, initialModels],
        ['lft_mid_goal', 'goal', black, lft, mid, zip, zip, initialModels],
        ['mid_mid_goal', 'goal', black, mid, mid, zip, zip, initialModels],
        ['rgt_mid_goal', 'goal', black, rgt, mid, zip, zip, initialModels],
        ['lft_dwn_goal', 'goal', black, lft, dwn, zip, zip, initialModels],
        ['mid_dwn_goal', 'goal', black, mid, dwn, zip, zip, initialModels],
        ['rgt_dwn_goal', 'goal', black, rgt, dwn, zip, zip, initialModels],
    ]

    for izone in izones:
        zone = GazeboZone()
        zone.name = izone[0]
        zone.type = izone[1]
        zone.color = izone[2]
        zone.x1 = izone[3]
        zone.y1 = izone[4]
        zone.x2 = izone[5]
        zone.y2 = izone[6]
        zone.models = izone[7]
        main.zones.zones.append(zone)

    imodels=[
        ['robot', white, 'field', 'robot', '', initialPose],
        ['ball',red,'field','red1', 'body',initialPose],
        ['ball',red,'field','red2', 'body',initialPose],
        ['ball',red,'field','red3', 'body',initialPose],
        ['ball',red,'mid_top_goal','red4', 'body',initialPose],
        ['ball',red,'mid_top_goal','red5', 'body',initialPose],
        ['ball',red,'mid_dwn_goal','red6', 'body',initialPose],
        ['ball',red,'lft_mid_goal','red7', 'body',initialPose],
        ['ball',red,'lft_top_goal','red8', 'body',initialPose],
        ['ball',red,'field','red9', 'body',initialPose],
        ['ball',red,'field','red10','body',initialPose],
        ['ball',red,'rgt_dwn_goal','red11','body',initialPose],
        ['ball',red,'rgt_mid_goal','red12','body',initialPose],
        ['ball',red,'rgt_top_goal','red13','body',initialPose],
        ['ball',red, 'lft_dwn_goal', 'red14', 'body', initialPose],
        ['ball', blue, 'field', 'blue1',  'body', initialPose],
        ['ball', blue, 'field', 'blue2',  'body', initialPose],
        ['ball', blue, 'field', 'blue3',  'body', initialPose],
        ['ball', blue, 'mid_top_goal', 'blue4',  'body', initialPose],
        ['ball', blue, 'mid_dwn_goal', 'blue5',  'body', initialPose],
        ['ball', blue, 'mid_dwn_goal', 'blue6',  'body', initialPose],
        ['ball', blue, 'lft_dwn_goal', 'blue7',  'body', initialPose],
        ['ball', blue, 'lft_top_goal', 'blue8',  'body', initialPose],
        ['ball', blue, 'rgt_dwn_goal', 'blue9',  'body', initialPose],
        ['ball', blue, 'rgt_mid_goal', 'blue10', 'body', initialPose],
        ['ball', blue, 'rgt_top_goal', 'blue11', 'body', initialPose],
        ['ball', blue, 'field', 'blue12', 'body', initialPose],
        ['ball', blue, 'field', 'blue13', 'body', initialPose],
        ['ball', blue, 'lft_mid_goal', 'blue14', 'body', initialPose],
    ]
    modelPoses=[
        ['robot', 0.633686721058, -1.39194882216, 0.0997087770738],
        ['red1', 0.0, 0.2487, 0.0800099999967],
        ['red2', 0.2487, 0.0, 0.0800099999967],
        ['red3', 0.0, 0.81375, 0.0800099999967],
        ['red4', 0.00146976659807, 1.63156428191, 0.396581261397],
        ['red5', 0.0240113762385, 1.62193593341, 0.0800271181202],
        ['red6', -0.00656466288878, -1.63940329292, 0.236740707394],
        ['red7', -1.61358286062, 0.0102068251477, 0.0800105462861],
        ['red8', -1.6134335244, 1.62682662319, 0.0800099999873],
        ['red9', -1.4393, -1.4393, 0.0800099999967],
        ['red10', -1.4393, 1.4393, 0.0800099999967],
        ['red11', 1.63792541721, -1.64526359704, 0.236744859064],
        ['red12', 1.63553671269, -0.00699725340991, 0.236746927086],
        ['red13', 1.63952621539, 1.63388809791, 0.236745095785],
        ['red14', -1.63282006258, -1.63646320493, 0.0800067739113],
        ['blue1', 0.0, -0.2487, 0.0800099999967],
        ['blue2', -0.2487, 0.0, 0.0800099999967],
        ['blue3', 0.0, -0.81375, 0.0800099999967],
        ['blue4', -0.00632284870405, 1.6329191691, 0.236755383602],
        ['blue5', 0.00123421530103, -1.64078638107, 0.396571427742],
        ['blue6', 0.0256863687412, -1.63940159521, 0.0800099999946],
        ['blue7', -1.64398654753, -1.64429012611, 0.239447645944],
        ['blue8', -1.64458043606, 1.63516331632, 0.236747437855],
        ['blue9', 1.61944346481, -1.61882679362, 0.0800099999927],
        ['blue10', 1.61483014229, 0.0177284115879, 0.0800099723762],
        ['blue11', 1.60739369411, 1.63674208941, 0.0800099311779],
        ['blue12', 1.4393, -1.4393, 0.0800099999967],
        ['blue13', 1.4393, 1.4393, 0.0800099999967],
        ['blue14', -1.64152449588, -0.00590554941898, 0.236747871432],
    ]

    for imodel in imodels:
        model = GazeboModel()
        model.type = imodel[0]
        model.color = imodel[1]
        model.zone = imodel[2]
        model.name = imodel[3]
        model.link = imodel[4]
        model.pose = imodel[5]
        main.models.models.append(model)

    for model in main.models.models:
        for modelPose in modelPoses:
            if(modelPose[0] == model.name):
                pose = Pose()
                pose.position.x = modelPose[1]
                pose.position.y = modelPose[2]
                pose.position.z = modelPose[3]
                model.pose = pose
                continue

#    print(types)
#    print(zones)
#    print(models)

    print("logic models")
    rospy.init_node('logic_models')

    typesPub = rospy.Publisher('/field/types', GazeboTypes, queue_size=10)
    zonesPub = rospy.Publisher('/field/zones', GazeboZones, queue_size=10)
    modelPub = rospy.Publisher('/field/models', GazeboModels, queue_size=10)

    def callback(imodels):
#        print("callback")
        main.models = imodels
#        for model in main.models.models:
#            if(model.name=='robot'):
#                print(model.pose.position)

    rospy.Subscriber('/gazebo/get_field', GazeboModels, callback)#(types, zones, models, typesPub, zonesPub, modelPub))

    def inCircle(x, y, center_x, center_y, radius):
        return (float(x) - float(center_x)) ** 2 + (float(y) - float(center_y)) ** 2 < float(radius) ** 2

    while not rospy.is_shutdown():
        try:
            time.sleep(.25)
            typesPub.publish(main.types)

#            print("[")
            for model in main.models.models:
#                print("    [" + model.name + ", " + str(model.pose.position.x) + ", " + str(model.pose.position.y) + ", " + str(model.pose.position.z) + "],")
                for zone in main.zones.zones:
#                    print(model.zone == zone.name)
                    if(model.zone == zone.name):
#                        print(zone.name)
                        index = -1
                        for i in range(0,len(zone.models.models)):
                            if(zone.models.models[i].name == model.name):
                                index = i
                        if(index==-1):
                            zone.models.models.append(model)
                        else:
                            zone.models.models[index] = model
                        continue
                zone.models.models = sorted(zone.models.models, key=lambda model: model.pose.position.z)
#            print("]")

            zonesPub.publish(main.zones)
            modelPub.publish(main.models)

        except rospy.ROSInterruptException:
            print("failed to display models")

if __name__ == '__main__':
    main()