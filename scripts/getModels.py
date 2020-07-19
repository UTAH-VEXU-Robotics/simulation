#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Bool
try:
    from driver.msg import Model, Models
except:
    print("driver not working")
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

    main.models = Models()

    # init publisher
    pub = rospy.Publisher('/gazebo/get_field', Models, queue_size=3)

    while not rospy.is_shutdown():
        try:
            # sleep
            rate.sleep()

            # wait for gazebo
            rospy.wait_for_service('/gazebo/get_model_state')
            # get updated models
            updated_models = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            out = Models()
            for model in main.models.models:
                imodel = Model()
                imodel.type = model.type
                imodel.color = model.color
                imodel.zone = model.zone
                imodel.name = model.name
                imodel.link = model.link
                imodel.pose = getPose(updated_models(str(imodel.name), str(imodel.link)))
                out.models.append(imodel)
            if(out != Models()):
                pub.publish(out)

        except rospy.ROSInterruptException:
            print("failed get gazebo models")

if __name__ == '__main__':
    print("get gazebo models")
    main()
