#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
from gazeboSimulation.msg import ChangeUp, ChangeUpGoals
import rospy

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class Tutorial:
    def __init__(self):
        rospy.init_node('simulation_world_node', anonymous=True)
        self.pub0 = rospy.Publisher('/field/state', ChangeUp, queue_size=3)
        self.pub1 = rospy.Publisher('/field/goals', ChangeUpGoals, queue_size=3)

    _red_balls = {
        'red_ball01': Block('red1',     'body' ),
        'red_ball02': Block('red2',     'body' ),
        'red_ball03': Block('red3',     'body' ),
        'red_ball04': Block('red4',     'body' ),
        'red_ball05': Block('red5',     'body' ),
        'red_ball06': Block('red6',     'body' ),
        'red_ball07': Block('red7',     'body' ),
        'red_ball08': Block('red8',     'body' ),
        'red_ball09': Block('red9',     'body' ),
        'red_ball10': Block('red10',    'body' ),
        'red_ball11': Block('red11',    'body' ),
        'red_ball12': Block('red12',    'body' ),
        'red_ball13': Block('red13',    'body' ),
    }
    _blue_balls = {
        'blue_ball01': Block('blue1',  'body'),
        'blue_ball02': Block('blue2',  'body'),
        'blue_ball03': Block('blue3',  'body'),
        'blue_ball04': Block('blue4',  'body'),
        'blue_ball05': Block('blue5',  'body'),
        'blue_ball06': Block('blue6',  'body'),
        'blue_ball07': Block('blue7',  'body'),
        'blue_ball08': Block('blue8',  'body'),
        'blue_ball09': Block('blue9',  'body'),
        'blue_ball10': Block('blue10', 'body'),
        'blue_ball11': Block('blue11', 'body'),
        'blue_ball12': Block('blue12', 'body'),
        'blue_ball13': Block('blue13', 'body'),
    }
    _robot = {
        'robot': Block('robot', 'base_link', ),
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

    def sortBalls(self):
        print("TODO: Sort balls to each goal")

    def publish_gazebos(self):
        rate = rospy.Rate(1)  # 1hz

        while True:
            try:
                model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                state = ChangeUp()
                goals = ChangeUpGoals()

                for block in self._robot.itervalues():
                    state.robot = self.getPose(model_coordinates(str(block._name), block._relative_entity_name))
                for block in self._red_balls.itervalues():
                    state.red_balls.poses.append(self.getPose(model_coordinates(str(block._name), block._relative_entity_name)))
                for block in self._blue_balls.itervalues():
                    state.blue_balls.poses.append(self.getPose(model_coordinates(str(block._name), block._relative_entity_name)))

                self.pub0.publish(state)



                self.pub1.publish(goals)

                rate.sleep()

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