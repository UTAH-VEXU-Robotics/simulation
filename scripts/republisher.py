#!/usr/bin/env python
import roslib
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from tf.msg import tfMessage
from geometry_msgs.msg import Twist

iodom  = Odometry()

def callback(msg):
    #print(msg.pose.pose)
    iodom = msg

def main():
    rospy.init_node('OdometryNode', anonymous=True)
    pub = rospy.Publisher('/simulation_robot/Odometry', Odometry, queue_size=10)
    rospy.Subscriber('/r2d2_diff_drive_controller/odom', Odometry, callback)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        #odom
        #rospy.loginfo(iodom)
        pub.publish(iodom)

        #sleep
        rate.sleep()

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()