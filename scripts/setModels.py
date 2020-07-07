#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from gazeboSimulation.msg import GazeboModel, GazeboModels
import rospy

def callback(model):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    state_msg = ModelState()
    state_msg.model_name = model.name
    state_msg.pose = model.pose

    rospy.wait_for_service('/gazebo/set_model_state')

    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

        print("Set state")

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def main():
    print("set gazebo models")
    rospy.init_node('set_pose')
    rospy.Subscriber("/gazebo/set_field", GazeboModel, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
