#!/usr/bin/env python
import rospy
from rospy import Time
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.srv import GetModelState, GetWorldProperties, GetModelProperties
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String, Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
#from navigation.msg import Field, Model, Type, Zone, Fields, Models, Types, Zones

def main():

    # init ros / gazebo
    rospy.init_node('get_models_node', anonymous=True)

    main.tf = TransformStamped

    main.markerPublisher = rospy.Publisher('/field/markers', MarkerArray,   queue_size=5)
#    main.fieldPublisher = rospy.Publisher('/field/state', Field,   queue_size=5)
#    main.fieldPublisher = rospy.Publisher('/field/state', Field,   queue_size=5)

#    main.model_states = rospy.ServiceProxy('gazebo/get_model_states', ModelStates)
#    main.world_properties = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
#    main.model_properties = rospy.ServiceProxy('gazebo/get_model_properties', GetModelProperties)

    def modelStateSubscriber(models):
        markers = MarkerArray()

        modelCnt = len(models.name)
        for i in range(0,modelCnt-1):
            ballScale = 0.16002
            if( "red" in models.name[i] ):
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = Time.now()
                marker.id = i
                marker.type = 2
                marker.action = 0
                marker.pose = models.pose[i]
                marker.scale.x = ballScale
                marker.scale.y = ballScale
                marker.scale.z = ballScale
                marker.color.r = 255
                marker.color.a = 1
                marker.text = models.name[i]
                markers.markers.append(marker)
            elif( "blue" in models.name[i] ):
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = Time.now()
                marker.id = i
                marker.type = 2
                marker.action = 0
                marker.pose = models.pose[i]
                marker.scale.x = ballScale
                marker.scale.y = ballScale
                marker.scale.z = ballScale
                marker.color.b = 255
                marker.color.a = 1
                marker.text = models.name[i]
                markers.markers.append(marker)

#        print(markers)
        main.markerPublisher.publish(markers)

    rospy.Subscriber('/gazebo/model_states', ModelStates, modelStateSubscriber)

    # set cycles per second
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            # sleep
            rate.sleep()


        except rospy.ROSInterruptException:
            print("failed get gazebo models")

if __name__ == '__main__':
    print("get gazebo models")
    main()
