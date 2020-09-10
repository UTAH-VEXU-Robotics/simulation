#!/usr/bin/env python
import rospy

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.srv import GetModelState, GetWorldProperties, GetModelProperties
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray

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

    main.tf = TransformStamped

    # set cycles per second
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            # sleep
            rate.sleep()

            # wait for gazebo
            rospy.wait_for_service('/gazebo/get_world_properties')

            world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            model_names = world().model_names

            br = tf.TransformBroadcaster()

#            print(model_names)

            for model_name in model_names:

                model = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
                body_names = model(model_name).body_names

#                print(model_name)

                for body_name in body_names:
                    find_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

                    state = find_state(model_name, body_name)



#                    br.sendTransform((state.twist.linear.x, state.twist.linear.y, state.twist.linear.z),
#                                     (state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w),
#                                     rospy.Time.now(),
#                                     body_name,
#                                     model_name)


        except rospy.ROSInterruptException:
            print("failed get gazebo models")

if __name__ == '__main__':
    print("get gazebo models")
    main()
