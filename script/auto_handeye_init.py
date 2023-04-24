#!/usr/bin/env python
# coding=utf-8

import rospy
from ur_move import MoveGroupPythonIntefaceTutorial



# back camera
#JOINT_START = [0.13159354031085968, -1.1898077170001429, 1.3657264709472656, -1.6672094503985804, 0.8086902499198914, -0.2972472349749964]
# front camera[0.13159354031085968, -1.1898077170001429, 1.3657264709472656, -1.6672094503985804, 0.8086902499198914, -0.2972472349749964]
#JOINT_START = [-0.676065746937887, -1.5574477354632776, 1.8398113250732422, -1.8189571539508265, 2.07348895072937, 0.8962278962135315]
# back
# JOINT_START = [0.55340576171875, -1.829555173913473, -1.8077607154846191, -1.378150300388672, -2.1659539381610315, 1.1426916122436523]
# front

# ready_state
# 0.0011715888977050781, -1.991136213342184, 2.278017822896139, -1.8571230373778285, -1.570937458668844, 0.2634305953979492

# [-0.026226345692769826, -2.029254738484518, -1.7494915167437952, -1.1875627676593226, -1.5812795797931116, 0.5651097893714905]
JOINT_START = [-0.026226345692769826, -2.029254738484518, -1.7494915167437952, -1.1875627676593226, -1.5812795797931116, 0.5651097893714905]
if __name__ == '__main__':

    NODE_NAME = 'easy_handeye_mover'

    rospy.init_node(NODE_NAME)

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