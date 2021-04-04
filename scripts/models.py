#!/usr/bin/env python
import rospy

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Twist, Pose
from gazebo_msgs.srv import GetModelState, GetWorldProperties, GetModelProperties, GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String, Bool, Float64, Int16


def field_callback(state):
    global game_state
    game_state = state

def robot_callback(state):
    global robot_state
    robot_state = state

def set_state(state):
    try:
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def main():
    # init ros
    rospy.init_node('models_node', anonymous=True)

    # set global vars, publishers, and subs
    global field_state_pub, field_state_set, set_model_state_pub, game_state, robot_state
    field_state_pub = rospy.Publisher('/game/field/state', Int16, queue_size=10)
    field_state_set = rospy.Publisher('/game/field/set_state', Int16, queue_size=10)
    field_state_pub = rospy.Publisher('/game/robot/state', Int16, queue_size=10)
    field_state_set = rospy.Publisher('/game/robot/set_state', Int16, queue_size=10)
    set_model_state_pub = rospy.Publisher('/game/set_model_state', ModelState, queue_size=10)
    rospy.Subscriber("/game/field/set_state", Int16, field_callback)
    rospy.Subscriber("/game/robot/set_state", Int16, robot_callback)
    rospy.Subscriber("/game/set_model_state", ModelState, set_state)

    # set the default states
    game_state = Int16()
    game_state.data = 3
    game_state_last = Int16()
    game_state_last.data = 0
    field_state_set.publish(game_state)

    robot_state = Int16()
    robot_state.data = 2
    robot_state_last = Int16()
    robot_state_last.data = 0
    field_state_set.publish(robot_state)

    # call init methods
    init_game_state_models()
    init_robot_state_models()

    while not rospy.is_shutdown():
        # print(str(game_state.data) + str(game_state_last.data))
        if game_state != game_state_last:
            if len(game_state_models) -1 >= game_state.data and game_state.data >= 0:
                print("new field state set: " + str(game_state.data))

                game_state_last = game_state
                for m in game_state_models[game_state.data]:
                    set_state(m)
            else:
                print("invalid field state number")

        if robot_state != robot_state_last:
            if len(robot_state_models) -1 >= robot_state.data and robot_state.data >= 0:
                print("new robot state set: " + str(robot_state.data))

                robot_state_last = robot_state
                for m in robot_state_models[robot_state.data]:
                    set_state(m)
            else:
                print("invalid robot state number")

    # # wait for gazebo
    # rospy.wait_for_service('/gazebo/get_world_properties')
    #
    # world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    # model_names = world().model_names
    #
    # br = tf.TransformBroadcaster()
    #
    # for model_name in model_names:
    #
    #     model = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    #     body_names = model(model_name).body_names
    #
    #     for body_name in body_names:
    #         find_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    #
    #         state = find_state(model_name, body_name)
    #         br.sendTransform((state.twist.linear.x, state.twist.linear.y, state.twist.linear.z),
    #                          (state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z,
    #                           state.pose.orientation.w),
    #                          rospy.Time.now(),
    #                          body_name,
    #                          model_name)


def init_robot_state_models():
    # set robot_state_models to global
    global robot_state_models

    # set the poses for all of the objects for a game
    robot_state_models, default_models, skill_top_models, skill_bottom_models, comp_red_top_models, comp_red_bottom_models, comp_blue_top_models, comp_blue_bottom_models  = [], [], [], [], [], [], [], []
    rp = 1.6; rn = -rp; p2 = .6096; n2 = -p2; p3 = .9144; n3 = -p3; p4 = 1.2192; n4 = -p4; op = 2; on = -op; r1 = .11;

    # robot_state 0 means that nothing is currently being set and is free to move
    robot_state_models.append([])

    model = ModelState(); model.model_name = 'robot';  model.reference_frame = 'world'; model.pose.position.x = p3; model.pose.position.y = p3; model.pose.position.z = r1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    robot_state_models.append(default_models)
    model = ModelState(); model.model_name = 'robot';  model.reference_frame = 'world'; model.pose.position.x = rn; model.pose.position.y = p3; model.pose.position.z = r1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skill_top_models.append(model)
    robot_state_models.append(skill_top_models)
    model = ModelState(); model.model_name = 'robot';  model.reference_frame = 'world'; model.pose.position.x = rn; model.pose.position.y = n3; model.pose.position.z = r1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skill_bottom_models.append(model)
    robot_state_models.append(skill_bottom_models)
    model = ModelState(); model.model_name = 'robot';  model.reference_frame = 'world'; model.pose.position.x = rn; model.pose.position.y = p3; model.pose.position.z = r1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_red_top_models.append(model)
    robot_state_models.append(comp_red_top_models)
    model = ModelState(); model.model_name = 'robot';  model.reference_frame = 'world'; model.pose.position.x = rn; model.pose.position.y = n3; model.pose.position.z = r1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_red_bottom_models.append(model)
    robot_state_models.append(comp_red_bottom_models)
    model = ModelState(); model.model_name = 'robot';  model.reference_frame = 'world'; model.pose.position.x = rp; model.pose.position.y = p3; model.pose.position.z = r1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_blue_top_models.append(model)
    robot_state_models.append(comp_blue_top_models)
    model = ModelState(); model.model_name = 'robot';  model.reference_frame = 'world'; model.pose.position.x = rp; model.pose.position.y = n3; model.pose.position.z = r1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_blue_bottom_models.append(model)
    robot_state_models.append(comp_blue_bottom_models)


def init_game_state_models():
    # set game_state_models to global
    global game_state_models

    # set the poses for all of the objects for a game
    game_state_models, default_models, comp_models, skills_models = [], [], [], []
    t0= 0; t1 = .08001; t2 = .34003; t3 = .50005; tp = 1.6275; tn = -tp; gp = 1.4393; gn = -gp; mp = .2487; mn = -mp; op = 2; on = -op;
    p2 = .6096; n2 = -p2; p3 = .9144; n3 = -p3; p4 = 1.2192; n4 = -p4; fp = 1.7488; fn = -fp;

    # game_state 0 means that nothing is currently being set and is free to move
    game_state_models.append([])

    # game_state 1 means default setup elements not in field: all the positions for all the objects for a field game state going by color, top to bottom, left to right, low to high
    model = ModelState(); model.model_name = 'blue1';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue2';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue3';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue4';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue5';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue6';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue7';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue8';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue9';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue10'; model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue11'; model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue12'; model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue13'; model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue14'; model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue15'; model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'blue16'; model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red1';   model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red2';   model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red3';   model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red4';   model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red5';   model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red6';   model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red7';   model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red8';   model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red9';   model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red10';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red11';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red12';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red13';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red14';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red15';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    model = ModelState(); model.model_name = 'red16';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; default_models.append(model)
    game_state_models.append(default_models)

    # game_state 2 means competition setup: all the positions for all the objects for a field game state going by color, top to bottom, left to right, low to high
    model = ModelState(); model.model_name = 'blue1';  model.reference_frame = 'world'; model.pose.position.x = tn; model.pose.position.y = tp; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue2';  model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = tp; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue3';  model.reference_frame = 'world'; model.pose.position.x = tp; model.pose.position.y = tp; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue4';  model.reference_frame = 'world'; model.pose.position.x = gp; model.pose.position.y = gp; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue5';  model.reference_frame = 'world'; model.pose.position.x = tn; model.pose.position.y = t0; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue6';  model.reference_frame = 'world'; model.pose.position.x = mn; model.pose.position.y = t0; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue7';  model.reference_frame = 'world'; model.pose.position.x = tp; model.pose.position.y = t0; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue8';  model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = mn; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue9';  model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = n3; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue10'; model.reference_frame = 'world'; model.pose.position.x = tn; model.pose.position.y = tn; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue11'; model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = tn; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue12'; model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = tn; model.pose.position.z = t3; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue13'; model.reference_frame = 'world'; model.pose.position.x = gp; model.pose.position.y = gn; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue14'; model.reference_frame = 'world'; model.pose.position.x = tp; model.pose.position.y = tn; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue15'; model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = p3; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'blue16'; model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = n3; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red1';   model.reference_frame = 'world'; model.pose.position.x = tn; model.pose.position.y = tp; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red2';   model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = tp; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red3';   model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = tp; model.pose.position.z = t3; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red4';   model.reference_frame = 'world'; model.pose.position.x = tp; model.pose.position.y = tp; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red5';   model.reference_frame = 'world'; model.pose.position.x = gn; model.pose.position.y = gp; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red6';   model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = p3; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red7';   model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = mp; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red8';   model.reference_frame = 'world'; model.pose.position.x = tn; model.pose.position.y = t0; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red9';   model.reference_frame = 'world'; model.pose.position.x = mp; model.pose.position.y = t0; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red10';  model.reference_frame = 'world'; model.pose.position.x = tp; model.pose.position.y = t0; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red11';  model.reference_frame = 'world'; model.pose.position.x = gn; model.pose.position.y = gn; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red12';  model.reference_frame = 'world'; model.pose.position.x = tn; model.pose.position.y = tn; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red13';  model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = tn; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red14';  model.reference_frame = 'world'; model.pose.position.x = tp; model.pose.position.y = tn; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red15';  model.reference_frame = 'world'; model.pose.position.x = on; model.pose.position.y = p3; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    model = ModelState(); model.model_name = 'red16';  model.reference_frame = 'world'; model.pose.position.x = on; model.pose.position.y = n3; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; comp_models.append(model)
    game_state_models.append(comp_models)

    # game_state 3 means skills setup: all the positions for all the objects for a field game state going by color, top to bottom, left to right, low to high
    model = ModelState(); model.model_name = 'blue1';  model.reference_frame = 'world'; model.pose.position.x = tn; model.pose.position.y = tp; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue2';  model.reference_frame = 'world'; model.pose.position.x = tn; model.pose.position.y = tp; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue3';  model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = tp; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue4';  model.reference_frame = 'world'; model.pose.position.x = tp; model.pose.position.y = tp; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue5';  model.reference_frame = 'world'; model.pose.position.x = tp; model.pose.position.y = tp; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue6';  model.reference_frame = 'world'; model.pose.position.x = tn; model.pose.position.y = t0; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue7';  model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = t0; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue8';  model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = t0; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue9';  model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = t0; model.pose.position.z = t3; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue10'; model.reference_frame = 'world'; model.pose.position.x = tp; model.pose.position.y = t0; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue11'; model.reference_frame = 'world'; model.pose.position.x = tn; model.pose.position.y = tn; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue12'; model.reference_frame = 'world'; model.pose.position.x = tn; model.pose.position.y = tn; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue13'; model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = tn; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue14'; model.reference_frame = 'world'; model.pose.position.x = tp; model.pose.position.y = tn; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue15'; model.reference_frame = 'world'; model.pose.position.x = tp; model.pose.position.y = tn; model.pose.position.z = t2; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'blue16'; model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red1';   model.reference_frame = 'world'; model.pose.position.x = n3; model.pose.position.y = fp; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red2';   model.reference_frame = 'world'; model.pose.position.x = p3; model.pose.position.y = fp; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red3';   model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = p4; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red4';   model.reference_frame = 'world'; model.pose.position.x = n4; model.pose.position.y = p3; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red5';   model.reference_frame = 'world'; model.pose.position.x = p4; model.pose.position.y = p3; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red6';   model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = p2; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red7';   model.reference_frame = 'world'; model.pose.position.x = n2; model.pose.position.y = t0; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red8';   model.reference_frame = 'world'; model.pose.position.x = p2; model.pose.position.y = t0; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red9';   model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = n2; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red10';  model.reference_frame = 'world'; model.pose.position.x = n4; model.pose.position.y = n3; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red11';  model.reference_frame = 'world'; model.pose.position.x = p4; model.pose.position.y = n3; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red12';  model.reference_frame = 'world'; model.pose.position.x = t0; model.pose.position.y = n4; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red13';  model.reference_frame = 'world'; model.pose.position.x = n3; model.pose.position.y = fn; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red14';  model.reference_frame = 'world'; model.pose.position.x = p3; model.pose.position.y = fn; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red15';  model.reference_frame = 'world'; model.pose.position.x = on; model.pose.position.y = t0; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    model = ModelState(); model.model_name = 'red16';  model.reference_frame = 'world'; model.pose.position.x = op; model.pose.position.y = op; model.pose.position.z = t1; model.pose.orientation.x = 0; model.pose.orientation.y = 0; model.pose.orientation.z = 0; model.pose.orientation.w = 1; skills_models.append(model)
    game_state_models.append(skills_models)


if __name__ == '__main__':
    main()
