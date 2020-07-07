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
from gazeboSimulation.msg import GazeboModel, GazeboModels
from tf.transformations import euler_from_quaternion, quaternion_from_euler

fieldPixels = 1000
fieldConvIn = fieldPixels/140.5
size = width, height = fieldPixels, fieldPixels
speed = [2, 2]
black = (0, 0, 0)
grey = (100, 100, 100)
white = (255, 255, 255)
red = (255,0,0)
blue = (0,0,255)
models = GazeboModels()
pub = rospy.Publisher('/gazebo/set_field', GazeboModel, queue_size=10)

def inchesToPixels(inches):
    out = inches * fieldConvIn
##    print("inches to pixels: " + str(inches) + ", " + str(out))
    return int(out)

def meterToPixels(meter):
    out = meter * 39.3701 * fieldConvIn
##    print("meter to pixels: "+ str(meter) + ", " + str(out))
    return int(out)

def pixelsToMeter(pixels):
    out = pixels / (39.3701 * fieldConvIn)
    print("pixels to meter: "+ str(pixels) + ", " + str(out))
    return float(out)


def xyToOriginPoint(x,y):
    ix=x+fieldPixels/2
    iy=-y+fieldPixels/2
    return (int(ix),int(iy))

def text(text, point,size=20):
    font = pygame.font.Font(pygame.font.get_default_font(),size)
    textSurface = font.render(text, True, white)
    TextSurf, TextRect = textSurface, textSurface.get_rect()
    TextRect.center = point
    screen.blit(TextSurf, TextRect)
    pygame.display.update()

robotRadius = meterToPixels(.3)

def inCircle(x,y,center_x, center_y):
    pygame.draw.circle(screen, white, (x,y), 5, 0)
    pygame.draw.circle(screen, black, (center_x,center_y), robotRadius, 1)
    return (x - center_x)**2 + (y - center_y)**2 < robotRadius**2

def drawModel(model,point):
    if (model.type == 'ball'):
        radius = meterToPixels(0.08001)
        if (model.spec == 'red'):
            pygame.draw.circle(screen, red, point, radius, 0)
        elif (model.spec == 'blue'):
            pygame.draw.circle(screen, blue, point, radius, 0)
    if(model.type == 'robot'):
        radius = robotRadius
        robotPoint = point
        if(model.spec == 'us'):
            pygame.draw.circle(screen, white, robotPoint, radius, 1)

class Zone:
    def __init__(self):
        self.name   = [] #n
        self.types  = [] #t
        self.x1     = [] #x1
        self.y1     = [] #y1
        self.x2     = [] #x2
        self.y2     = [] #y2
        self.radius = [] #r
        self.models  = GazeboModels()
    def define(self,n,t,x1,y1,x2,y2,r=0):
        self.name   = n
        self.types  = t
        self.x1     = x1
        self.y1     = y1
        self.x2     = x2
        self.y2     = y2
        self.radius = r

    def addModel(self,model):
        self.models.models.append(model)

    def draw(self,screen,width=1):
        if (self.types == 'circle'):
            point = xyToOriginPoint(self.x1, self.y1)
            pygame.draw.circle(screen, black, point, self.radius,width)
            if(len(self.models.models)!=0):
                modelToDraw = self.models.models[0]
                for model in self.models.models:
                    if(model.pose.position.z>modelToDraw.pose.position.z):
                        modelToDraw = model
                drawModel(modelToDraw,point)
        elif(self.types == 'rect'):
            size = (self.x2-self.x1,self.y2-self.y1)
            rect = pygame.Rect(xyToOriginPoint(self.x1,-self.y1),size)
            pygame.draw.rect(screen,grey,rect)

class Field:
    def __init__(self, screen):
        self.screen = screen
        self.states = [
            # name           #types     #x1      #y1      #x2   #y2    #radius
            ['field', 'rect', -1.82, 1.82, 1.82, -1.82, 0],
            ['lft_top_goal', 'circle', -1.6335, 1.6335, 0, 0, 0.130048],
            ['mid_top_goal', 'circle', 0, 1.6335, 0, 0, 0.130048],
            ['rgt_top_goal', 'circle', 1.6335, 1.6335, 0, 0, 0.130048],
            ['lft_mid_goal', 'circle', -1.6335, 0, 0, 0, 0.130048],
            ['mid_mid_goal', 'circle', 0, 0, 0, 0, 0.130048],
            ['rgt_mid_goal', 'circle', 1.6335, 0, 0, 0, 0.130048],
            ['lft_dwn_goal', 'circle', -1.6335, -1.6335, 0, 0, 0.130048],
            ['mid_dwn_goal', 'circle', 0, -1.6335, 0, 0, 0.130048],
            ['rgt_dwn_goal', 'circle', 1.6335, -1.6335, 0, 0, 0.130048],
        ]

    def redraw(self,imodels):
        self.screen.fill(white)
        for s in self.states:
            zone = Zone()
            zone.define(s[0], s[1], meterToPixels(s[2]), meterToPixels(s[3]), meterToPixels(s[4]), meterToPixels(s[5]), meterToPixels(s[6]))

            if imodels.models:
                for model in imodels.models:
                    if(model.name == 'robot'):
                        robotPoint = xyToOriginPoint(meterToPixels(model.pose.position.x),
                                                meterToPixels(model.pose.position.y))
                        gazeboRobotPoint = (model.pose.position.x,model.pose.position.y,model.pose.position.z)
                        zonePoint = gazeboRobotPoint
#                        print(zone.x1,zone.y1)
#                        print( (zone.x1 - robotPoint[0])**2 + (zone.y1 -robotPoint[1])**2, '<', robotRadius**2)
                        if( inCircle( zone.x1,zone.y1,robotPoint[0],robotPoint[1] ) ):
                            print("zone in robot radius")
                            zonePoint = (s[2],s[3],1)
                        continue

                for model in imodels.models:
                    point = xyToOriginPoint(meterToPixels(model.pose.position.x), meterToPixels(model.pose.position.y))
                    if(model.state == zone.name):
                        zone.addModel(model)
                    if(model.state == 'field'):
                        drawModel(model,point)
                    if( inCircle( point[0],point[1],robotPoint[0],robotPoint[1] )):
                        if( model.name == 'robot' ): continue
                        imodel = mode
                        imodel.pose.position.x = gazeboRobotPoint[0]
                        imodel.pose.position.y = gazeboRobotPoint[1]
                        imodel.pose.position.z = gazeboRobotPoint[2] + .1

                        # if gazeboRobotPoint != zonePoint then set the model to zonePoint

                        pub.publish(imodel)

            zone.draw(self.screen, meterToPixels(0.054229))

def callback(imodels):
    models = imodels
    field.redraw(models)

pygame.init()
pygame.display.set_caption('Field')
screen = pygame.display.set_mode(size)
field = Field(screen)

def main():
    print("display models")
    rospy.init_node('display_models')

    rospy.Subscriber("/gazebo/get_field", GazeboModels, callback)
    # init publisher


    while not rospy.is_shutdown():
        try:
            time.sleep(.25)
            pygame.display.flip()

        except rospy.ROSInterruptException:
            print("failed to display models")


#        for event in pygame.event.get():
#            if event.type == pygame.QUIT: sys.exit()
#        field.redraw()
#        pygame.display.flip()


if __name__ == '__main__':
    main()