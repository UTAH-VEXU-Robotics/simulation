#!/usr/bin/env python
import roslib
import rospy
import pygame, sys
import numpy
import time
import math
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose,Twist
from std_msgs.msg import String, Float32, ColorRGBA, Bool, Int16, Int8
from gazeboSimulation.msg import GazeboModel, GazeboModels, GazeboType, GazeboTypes, GazeboZone, GazeboZones, ChangeUpGoal, ChangeUpGoals, ChangeUpField, Action, Actions, Task, Tasks

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
    # subscriber objects
    main.types = GazeboTypes()
    main.zones = GazeboZones()
    main.models = GazeboModels()
    main.field = ChangeUpField()

    # publisher objects
    main.tasks = Tasks()

    main.actions = Actions()

    print("cost models")
    rospy.init_node('cost_models')

    pub = rospy.Publisher('/field/task', Tasks, queue_size=3)

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

    def nonZeroFloat(ia):
        a = ia
        if(a == 0):
            a = .000001
        return float(a)

    def inCircle(x, y, center_x, center_y, radius):
        return (float(x) - float(center_x)) ** 2 + (float(y) - float(center_y)) ** 2 < float(radius) ** 2

    def distToCost(distance):
        metersPerSecond = 2.5 * .5
        return float(distance)/(metersPerSecond)

    def distBetweenPoints(ia,ib):
        return math.sqrt((nonZeroFloat(ib[0]) - nonZeroFloat(ia[0]))**2 + (nonZeroFloat(ib[1]) - nonZeroFloat(ia[1])) ** 2)

    def findZone(name):
        for obj in main.zones.zones:
            if(obj.name == name):
                return obj
        print("zone not found!")

    def findModel(name):
        for obj in main.models.models:
            if(obj.name == name):
                return obj
        print("model not found!")

    def findModelInZone(modelName,zoneName):
        zone = findZone(zoneName)
        for model in zone.models.models:
            if(model.name == modelName):
                return model
        print("model is not in zone!")

    def findGoalInField(name):
        for obj in main.field.goals.goals:
            if(obj.zoneName == name):
                return obj
        print("goal is not in field!")

    def findModelInGoalInField(modelName,goalName):
        goal = findGoalInField(goalName)
        for name in goal.modelNames:
            if(name == modelName):
                return findModel(name)
        print("model is not in a goal!")

    def findNearestBall(color):
#        print(color)

        nearestZoneName = ""
        nearestModelName = ""
        nearestBallDist = Float32(100.0)
        nearestCost = Float32(0)
        nearestReward = Int16(-100)

        for i in range(0,len(main.field.distanceFromRobot)):
#            print("len: ",len(main.field.modelColors)>i, len(main.field.modelColors), i)
            if(len(main.field.modelColors)>i):
                if(main.field.modelColors[i] == color):
                    if(main.field.distanceFromRobot[i] < nearestBallDist):
#                        print(main.field.distanceFromRobot[i], nearestBallDist)
                        nearestZoneName = main.field.fieldName
                        nearestModelName = main.field.modelNames[i]
                        nearestBallDist = main.field.distanceFromRobot[i]
                        nearestCost = Float32(distToCost(nearestBallDist.data))
                        nearestReward = Int16(0)
#        print(nearestZoneName, nearestModelName, nearestBallDist)

        goals = sorted(main.field.goals.goals, key=lambda goal: goal.distanceFromRobot.data)
        for goal in goals:
            if(len(goal.modelColors)>0):
                if(goal.modelColors[0] == color):
                    nearestZoneName  = goal.zoneName
                    nearestModelName = goal.modelNames[0]
                    nearestBallDist  = goal.distanceFromRobot
                    nearestCost = Float32(distToCost(nearestBallDist.data))
                    nearestReward = Int16(-1)
                    break
#        print(nearestZoneName, nearestModelName, nearestBallDist)

        action = Action()
        action.zoneName = nearestZoneName
        action.model = nearestModelName
        action.toZone = False
        action.cost = nearestCost
        action.reward = nearestReward
        action.newRobotPose = findModel(action.model).pose

        return action

    def takeBallFromGoal(actions, zoneName):
        goal = findGoalInField(zoneName)

        action = Action()
        action.zoneName = zoneName
        action.model = actions.actions[-1].model
        action.toZone = True
        robot = actions.actions[-1].newRobotPose
        zone = findZone(zoneName)
        action.cost = Float32(distToCost(distBetweenPoints( (zone.x1.data,zone.y1.data), (robot.position.x,robot.position.y) )))
        reward = 0
        #TODO: if an action will take away a row of 6 from the other alliance +6 to reward
        action.reward = Int16(reward)
        zone = findZone(zoneName)
        for model in zone.models.models:
            if(model.zone == zone.name):
#                print(zoneName, model.pose.position)
                action.newRobotPose = model.pose
                break

        return action

    def putBallInGoal(actions, zoneName):
        goal = findGoalInField(zoneName)

        action = Action()
        action.zoneName = zoneName
        action.model = actions.actions[-1].model
        action.toZone = True
        robot = actions.actions[-1].newRobotPose
#        print(robot)
        zone = findZone(zoneName)
        action.cost = Float32(distToCost(distBetweenPoints( (zone.x1.data,zone.y1.data), (robot.position.x,robot.position.y) )))
        reward = 6
        #TODO: if an action will take away a row of 6 from the other alliance +6 to reward
        action.reward = Int16(reward)
        for model in zone.models.models:
            if(model.zone == zone.name):
                action.newRobotPose = model.pose
                break

        return action

    noCost = Float32(0.0)
    noReward = Int16(0)

    while not rospy.is_shutdown():
        try:
            allianceColor = Int8()
            allianceColor.data = -1

            if(main.field != ChangeUpField()):
                for goal in main.field.goals.goals:
                    task = Task()
                    task.zoneName = goal.zoneName
                    task.colorOnTop = allianceColor
                    if(len(goal.modelColors)>0):
                        if(goal.modelColors[-1] == allianceColor):
#                            print(goal.zoneName,"same color on top")
                            task.actions = []
                            task.cost = noCost
                            task.reward = noReward
                        else:
#                            print(goal.zoneName,"not same color on top")
                            actions = Actions()
                            actions.actions.append(findNearestBall(allianceColor))
                            actions.actions.append(takeBallFromGoal(actions,task.zoneName))
                            actions.actions.append(putBallInGoal(actions,task.zoneName))

                            task.actions = actions

                            for action in task.actions.actions:
                                task.cost = Float32(task.cost.data + action.cost.data)
                                task.reward = Int16(task.reward.data + action.reward.data)



#                            print(task)
#                    task.actions.actions = sorted(task.actions.actions, key=lambda action: action.reward)
                    main.tasks.tasks.append(task)

#            print(main.tasks.tasks)

            bestTask = Task()
            bestTask.cost = Float32(100)
            bestTask.reward = Int16(-100)

            for task in main.tasks.tasks:
                bestRating = bestTask.cost.data / (bestTask.cost.data + .00001)
                taskRating = task.cost.data / (task.reward.data + .00001)
                if(taskRating != 0):
                    if( taskRating < bestRating ):
                        bestTask = task

            print(bestTask)

                        


            time.sleep(.25)

        except rospy.ROSInterruptException:
            print("failed to display models")

if __name__ == '__main__':
    main()
