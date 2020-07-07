#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from gazeboSimulation.msg import GazeboModel, GazeboModels
import rospy

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

def main():
    # init ros / gazebo
    rospy.init_node('get_models_node', anonymous=True)

    # set cycles per second
    rate = rospy.Rate(10)

    main.models = GazeboModels()

    def callback(models):
        main.models = models

    rospy.Subscriber('/field/models', GazeboModels, callback)

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
            for model in main.models.models:
                imodel = GazeboModel()
                imodel.type = model.type
                imodel.color = model.color
                imodel.zone = model.zone
                imodel.name = model.name
                imodel.link = model.link
                imodel.pose = getPose(updated_models(str(imodel.name), str(imodel.link)))
                out.models.append(imodel)

            if(out != GazeboModels()):
#                print(out)
                pub.publish(out)

        except rospy.ROSInterruptException:
            print("failed get gazebo models")

if __name__ == '__main__':
    print("get gazebo models")
    main()
