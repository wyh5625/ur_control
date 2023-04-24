#!/usr/bin/env python
# coding=utf-8

import rospy
from ur_move import MoveGroupPythonIntefaceTutorial
from controller_manager_msgs.srv import SwitchController



# back camera
# JOINT_START = [0.041345950216054916, -1.3676288763629358, 1.5925488471984863, -1.7650192419635218, 1.0725597143173218, -0.1923592726336878]
# front camera
# JOINT_START = [-0.8125880400287073, -1.60964280763735, 1.492295742034912, -1.4517219702350062, -1.579724136983053, 0.1262204796075821]

JOINT_START = [-0.011487785969869435, -2.044453283349508, -1.5493583679199219, -1.4955685895732422, -1.5396698156939905, -0.1436074415790003]
if __name__ == '__main__':

    NODE_NAME = 'easy_handeye_mover'

    rospy.init_node(NODE_NAME)

    rospy.wait_for_service('/controller_manager/switch_controller')

    try:
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        ret = switch_controller('scaled_pos_traj_controller', 'joint_group_vel_controller', 1, True, 2)
    except rospy.ServiceException as e:
        print("Service call failed!")

    ur_control = MoveGroupPythonIntefaceTutorial()
    ur_control.group.set_max_velocity_scaling_factor(0.1)
    ur_control.group.set_max_acceleration_scaling_factor(0.2)
    rospy.sleep(1)

    ur_control.remove_allobjects()
    ur_control.group.get_current_joint_values()

    ur_control.go_to_joint_state(*JOINT_START)

    # # ee_link
    # start_pose = init_pose

    # # tool0_controller
    # current_plan = ur_control.plan_to_pose(start_pose)
    # ur_control.group.execute(current_plan)