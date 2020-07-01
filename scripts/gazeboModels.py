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

    def sortBalls(self, ball, name, identity):
        print("TODO: Sort balls to each goal")
#        if( ball.state.x**2 + ball.state.y**2 < .16**2 ):



    def publish_gazebos(self):
        rospy.init_node('simulation_world_node', anonymous=True)
        rate = rospy.Rate(1)
        self.pub0 = rospy.Publisher('/field/state', ChangeUp, queue_size=3)
        self.pub1 = rospy.Publisher('/field/goals', ChangeUpGoals, queue_size=3)

        rospy.wait_for_service('/gazebo/get_model_state')

        while not rospy.is_shutdown():
            try:
                rate.sleep()
                rospy.wait_for_service('/gazebo/get_model_state')
                self.model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                state = ChangeUp()
                goals = ChangeUpGoals()

                for block in self._robot.itervalues():
                    state.robot = self.getPose(self.model_coordinates(str(block._name), block._relative_entity_name))
                for block in self._red_balls.itervalues():
                    model = self.getPose(self.model_coordinates(str(block._name), block._relative_entity_name))
                    self.sortBalls(model,str(block._name),block._relative_entity_name)
                    state.red_balls.poses.append(model)
                for block in self._blue_balls.itervalues():
                    state.blue_balls.poses.append(self.getPose(self.model_coordinates(str(block._name), block._relative_entity_name)))
                self.pub0.publish(state)


#                for ball in state.red_balls.itervalues():
#                    self.sortBalls(ball)


                self.pub1.publish(goals)


            except rospy.ROSInterruptException:
                print("failed gazebo models")

if __name__ == '__main__':
    print("gazeboModels")
    tuto = Tutorial()
    tuto.publish_gazebos()
