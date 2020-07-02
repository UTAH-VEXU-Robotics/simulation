#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from gazeboSimulation.msg import GazeboModel, GazeboModels
import rospy

class Zone:
    def __init__(self):
        self.name   = [] #n
        self.types  = [] #t
        self.x1     = [] #x1
        self.y1     = [] #y1
        self.x2     = [] #x2
        self.y2     = [] #y2
        self.radius = [] #r

    def define(self,n,t,x1,y1,x2,y2,r):
        self.name   = n
        self.types  = t
        self.x1     = x1
        self.y1     = y1
        self.x2     = x2
        self.y2     = y2
        self.radius = r

    def inZone(self,model):
        if(model.state != self.name):
            if (self.types == 'rect'):
                return (self.x1 < model.pose.position.x < self.x2) & (self.y1 < model.pose.position.y < self.y2)
            elif (self.types == 'circle'):
    #            print(model.pose.position.x - self.x1) ** 2 + (model.pose.position.y - self.y1) ** 2 < self.radius ** 2
                return (model.pose.position.x - self.x1) ** 2 + (model.pose.position.y - self.y1) ** 2 < self.radius ** 2
        else:
            return False

def getPose(ipose):
    pose = Pose()
    pose.position.x = ipose.pose.position.x
    pose.position.y = ipose.pose.position.y
    pose.position.z = ipose.pose.position.z
    pose.orientation.x = ipose.pose.orientation.x
    pose.orientation.y = ipose.pose.orientation.y
    pose.orientation.z = ipose.pose.orientation.z
    pose.orientation.w = ipose.pose.orientation.w
    return pose

def findInArray(list, elem):
    list_row = [list.index(row) for row in list if elem in row]
    return elem[list_row]

def updateModelState(model):
    states = [
         #name           #types     #x1      #y1      #x2   #y2    #radius
#        ['not_in_field', 'circle', 0      , 0     ,  0   , 0    , 100],
        ['field'       , 'rect'  , -1.82  , 1.82  ,  1.82, -1.82, 0  ],
        ['lft_top_goal', 'circle', -1.6335, 1.6335 , 0   , 0    , .05],
        ['mid_top_goal', 'circle', 0      , 1.6335 , 0   , 0    , .05],
        ['rgt_top_goal', 'circle', 1.6335 , 1.6335 , 0   , 0    , .05],

        ['lft_mid_goal', 'circle', -1.6335, 0      , 0   , 0    , .05],
        ['mid_mid_goal', 'circle', 0      , 0      , 0   , 0    , .05],
        ['rgt_mid_goal', 'circle', 1.6335 , 0      , 0   , 0    , .05],

        ['lft_dwn_goal', 'circle', -1.6335, -1.6335, 0   , 0    , .05],
        ['mid_dwn_goal', 'circle', 0      , -1.6335, 0   , 0    , .05],
        ['rgt_dwn_goal', 'circle', 1.6335 , -1.6335, 0   , 0    , .05],
    ]

    if(model.type == 'robot'): pass
    elif(model.type == 'ball'):
        for s in states:
            zone = Zone()
            zone.define(s[0],s[1],s[2],s[3],s[4],s[5],s[6])
            if(zone.inZone(model)):
                model.state = zone.name
#                print("model name: " + model.name)
#                print("model state: " + model.state)
#                print("zone name: " + zone.name)

def main():
    # models is an array of GazeboModels
    models = GazeboModels()

    # the initial pose is just a default pose that will change later
    initialPose = Pose()

    # intialize all the objects here. see GazeboModels.msg for more details
    objects=[
        ## four robots
        #### us
        ['robot', 'us', 'field', 'robot', 'base_link', initialPose],
        #['robot', 'us', 'field', 'robot2', 'base_link', initialPose],
        #### them
        #['robot', 'them', 'field', 'robot3', 'base_link', initialPose],
        #['robot', 'them', 'field', 'robot4', 'base_link', initialPose],

        ## balls
        #### red
        ['ball','red','field','red1', 'body',initialPose],
        ['ball','red','field','red2', 'body',initialPose],
        ['ball','red','field','red3', 'body',initialPose],
        ['ball','red','mid_top_goal','red4', 'body',initialPose],
        ['ball','red','mid_top_goal','red5', 'body',initialPose],
        ['ball','red','mid_dwn_goal','red6', 'body',initialPose],
        ['ball','red','lft_mid_goal','red7', 'body',initialPose],
        ['ball','red','lft_top_goal','red8', 'body',initialPose],
        ['ball','red','field','red9', 'body',initialPose],
        ['ball','red','field','red10','body',initialPose],
        ['ball','red','rgt_dwn_goal','red11','body',initialPose],
        ['ball','red','rgt_mid_goal','red12','body',initialPose],
        ['ball','red','rgt_top_goal','red13','body',initialPose],
        ['ball', 'red', 'lft_dwn_goal', 'red14', 'body', initialPose],

        #### blue
        ['ball', 'blue', 'field', 'blue1',  'body', initialPose],
        ['ball', 'blue', 'field', 'blue2',  'body', initialPose],
        ['ball', 'blue', 'field', 'blue3',  'body', initialPose],
        ['ball', 'blue', 'mid_top_goal', 'blue4',  'body', initialPose],
        ['ball', 'blue', 'mid_dwn_goal', 'blue5',  'body', initialPose],
        ['ball', 'blue', 'mid_dwn_goal', 'blue6',  'body', initialPose],
        ['ball', 'blue', 'lft_dwn_goal', 'blue7',  'body', initialPose],
        ['ball', 'blue', 'lft_top_goal', 'blue8',  'body', initialPose],
        ['ball', 'blue', 'rgt_dwn_goal', 'blue9',  'body', initialPose],
        ['ball', 'blue', 'rgt_mid_goal', 'blue10', 'body', initialPose],
        ['ball', 'blue', 'rgt_top_goal', 'blue11', 'body', initialPose],
        ['ball', 'blue', 'field', 'blue12', 'body', initialPose],
        ['ball', 'blue', 'field', 'blue13', 'body', initialPose],
        ['ball', 'blue', 'lft_mid_goal', 'blue14', 'body', initialPose],
    ]

    # pass objects to models
    for object in objects:
        model = GazeboModel()

        model.type =  str(object[0])
        model.spec =  str(object[1])
        model.state = str(object[2])
        model.name =  str(object[3])
        model.link =  str(object[4])
        model.pose =  object[5]
        models.models.append(model)

    # init ros / gazebo
    rospy.init_node('get_models_node', anonymous=True)

    # set cycles per second
    rate = rospy.Rate(10)

    # init publisher
    pub = rospy.Publisher('/gazebo/get_field', GazeboModels, queue_size=5)

    while not rospy.is_shutdown():
        try:
            # sleep
            rate.sleep()

            # wait for gazebo
            rospy.wait_for_service('/gazebo/get_model_state')

            # get updated models
            updated_models = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

            out = GazeboModels()
            for model in models.models:
                imodel = GazeboModel()
                imodel.type = model.type
                imodel.spec = model.spec
                imodel.state = model.state
                imodel.name = model.name
                imodel.link = model.link
                imodel.pose = getPose(updated_models(str(imodel.name), str(imodel.link)))
                updateModelState(imodel)
                out.models.append(imodel)

            pub.publish(out)

        except rospy.ROSInterruptException:
            print("failed get gazebo models")

if __name__ == '__main__':
    print("get gazebo models")
    main()
