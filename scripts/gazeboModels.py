#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
import rospy

class Block:
    def __init__(self, name, relative_entity_name, pub):
        self._name = name
        self._relative_entity_name = relative_entity_name
        self._pub = pub

class Tutorial:

    def __init__(self):
        rospy.init_node('simulation_world_node', anonymous=True)

    _models = {
        'robot': Block(     'robot','base_link',rospy.Publisher('/gazebo/model/robot/01',     Pose, queue_size=1)),
        'red_ball01': Block('red1',     'body', rospy.Publisher('/gazebo/model/red_ball/01',  Pose, queue_size=1) ),
        'red_ball02': Block('red2',     'body', rospy.Publisher('/gazebo/model/red_ball/02',  Pose, queue_size=1) ),
        'red_ball03': Block('red3',     'body', rospy.Publisher('/gazebo/model/red_ball/03',  Pose, queue_size=1) ),
        'red_ball04': Block('red4',     'body', rospy.Publisher('/gazebo/model/red_ball/04',  Pose, queue_size=1) ),
        'red_ball05': Block('red5',     'body', rospy.Publisher('/gazebo/model/red_ball/05',  Pose, queue_size=1) ),
        'red_ball06': Block('red6',     'body', rospy.Publisher('/gazebo/model/red_ball/06',  Pose, queue_size=1) ),
        'red_ball07': Block('red7',     'body', rospy.Publisher('/gazebo/model/red_ball/07',  Pose, queue_size=1) ),
        'red_ball08': Block('red8',     'body', rospy.Publisher('/gazebo/model/red_ball/08',  Pose, queue_size=1) ),
        'red_ball09': Block('red9',     'body', rospy.Publisher('/gazebo/model/red_ball/09',  Pose, queue_size=1) ),
        'red_ball10': Block('red10',    'body', rospy.Publisher('/gazebo/model/red_ball/10',  Pose, queue_size=1) ),
        'red_ball11': Block('red11',    'body', rospy.Publisher('/gazebo/model/red_ball/11',  Pose, queue_size=1) ),
        'red_ball12': Block('red12',    'body', rospy.Publisher('/gazebo/model/red_ball/12',  Pose, queue_size=1) ),
        'red_ball13': Block('red13',    'body', rospy.Publisher('/gazebo/model/red_ball/13',  Pose, queue_size=1) ),
        'blue_ball01': Block('blue1',   'body', rospy.Publisher('/gazebo/model/blue_ball/01', Pose, queue_size=1) ),
        'blue_ball02': Block('blue2',   'body', rospy.Publisher('/gazebo/model/blue_ball/02', Pose, queue_size=1) ),
        'blue_ball03': Block('blue3',   'body', rospy.Publisher('/gazebo/model/blue_ball/03', Pose, queue_size=1) ),
        'blue_ball04': Block('blue4',   'body', rospy.Publisher('/gazebo/model/blue_ball/04', Pose, queue_size=1) ),
        'blue_ball05': Block('blue5',   'body', rospy.Publisher('/gazebo/model/blue_ball/05', Pose, queue_size=1) ),
        'blue_ball06': Block('blue6',   'body', rospy.Publisher('/gazebo/model/blue_ball/06', Pose, queue_size=1) ),
        'blue_ball07': Block('blue7',   'body', rospy.Publisher('/gazebo/model/blue_ball/07', Pose, queue_size=1) ),
        'blue_ball08': Block('blue8',   'body', rospy.Publisher('/gazebo/model/blue_ball/08', Pose, queue_size=1) ),
        'blue_ball09': Block('blue9',   'body', rospy.Publisher('/gazebo/model/blue_ball/09', Pose, queue_size=1) ),
        'blue_ball10': Block('blue10',  'body', rospy.Publisher('/gazebo/model/blue_ball/10', Pose, queue_size=1) ),
        'blue_ball11': Block('blue11',  'body', rospy.Publisher('/gazebo/model/blue_ball/11', Pose, queue_size=1) ),
        'blue_ball12': Block('blue12',  'body', rospy.Publisher('/gazebo/model/blue_ball/12', Pose, queue_size=1) ),
        'blue_ball13': Block('blue13',  'body', rospy.Publisher('/gazebo/model/blue_ball/13', Pose, queue_size=1) ),
    }

    def getPose(self, _resp_coordinates):
        ik_pose = Pose()
        ik_pose.position.x = _resp_coordinates.pose.position.x
        ik_pose.position.y = _resp_coordinates.pose.position.y
        ik_pose.position.z = _resp_coordinates.pose.position.z
        ik_pose.orientation.x = _resp_coordinates.pose.orientation.x
        ik_pose.orientation.y = _resp_coordinates.pose.orientation.y
        ik_pose.orientation.z = _resp_coordinates.pose.orientation.z
        ik_pose.orientation.w = _resp_coordinates.pose.orientation.w
        return ik_pose

    def publish_gazebos(self):
        while not rospy.is_shutdown():
            try:
                model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                for block in self._models.itervalues():
                    pose = self.getPose(model_coordinates(str(block._name), block._relative_entity_name))
                    block._pub.publish(pose)

            except rospy.ServiceException as e:
                rospy.loginfo("Get Model State service call failed:  {0}".format(e))

#    def show_gazebos(self):
#        try:
#            model_coordinates = rospy.ServiceProxy('/gazebo/get_state', GetModelState)
#            for block in self._blockListDict.itervalues():
#                blockName = str(block._name)
#                resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
#                print('\n')
#                print("Status.success = ", resp_coordinates.success)
#                print(blockName)
#                print("Model " + str(block._name))
#                print("Valeur de X : " + str(resp_coordinates.pose.position.x))
#                print("Valeur de Y : " + str(resp_coordinates.pose.position.y))
#                print("Valeur de Z : " + str(resp_coordinates.pose.position.z))
#                print("Quaternion X : " + str(resp_coordinates.pose.orientation.x))
#                print("Quaternion Y : " + str(resp_coordinates.pose.orientation.y))
#                print("Quaternion Z : " + str(resp_coordinates.pose.orientation.z))
#        except rospy.ServiceException as e:
#            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


if __name__ == '__main__':
    print("gazeboModels")
    tuto = Tutorial()
    tuto.publish_gazebos()