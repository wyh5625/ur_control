#!/usr/bin/env python
# coding=utf-8
from __future__ import print_function

import math
import os
import sys
sys.path.append('..')

from copy import deepcopy
from itertools import chain

import rospy
from geometry_msgs.msg import Quaternion, geometry_msgs
from moveit_commander import MoveGroupCommander
import numpy as np

from moveit_commander.conversions import pose_to_list
from tf import transformations
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_matrix, euler_from_quaternion

from ur_control.helper import save_file, transform_to_mat44, mat44_to_xyzrpy, path_root
from ur_control.helper import gen_pose

from ur_move import MoveGroupPythonIntefaceTutorial

from tf.transformations import quaternion_from_euler

from handeye_client import HandeyeClient

import rospkg
rospack = rospkg.RosPack()
path_root = rospack.get_path('example_organization_ur_launch')

#path_root = os.path.abspath(os.path.join(os.path.abspath(os.path.dirname(__file__)), '../../'))
print(path_root)
path_base_cam_tf_pub = os.path.join(path_root, 'launch/base_cam_tf_pub.launch')


def add_tagbox(self, timeout=4):
    box_name = "tag_box"
    box_size = (0.15, 0.15, 0.01)
    quat = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "tool0"
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0 + box_size[2] / 2
    box_pose.pose.orientation.x = quat[0]
    box_pose.pose.orientation.y = quat[1]
    box_pose.pose.orientation.z = quat[2]
    box_pose.pose.orientation.w = quat[3]

    self.scene.add_box(box_name, box_pose, size=box_size)

    return self.wait_for_state_update(box_name, object_is_known=True, timeout=timeout)

def attach_tagbox(self, timeout=4):
    base_link = 'ee_link'
    box_name = "tag_box"
    self.scene.attach_box(base_link, box_name, touch_links=[base_link])
    return self.wait_for_state_update(box_name, object_is_attached=True, object_is_known=False, timeout=timeout)

# back camera
#JOINT_START = [0.13159354031085968, -1.1898077170001429, 1.3657264709472656, -1.6672094503985804, 0.8086902499198914, -0.2972472349749964]
# front camera
#JOINT_START = [-0.676065746937887, -1.5574477354632776, 1.8398113250732422, -1.8189571539508265, 2.07348895072937, 0.8962278962135315]
#init_pose = gen_pose(-0.14977545397, -0.651859335945, 0.238756893858, 0.630014979713, -0.528294020241, 0.469421740715, 0.321915800885)
#camera0  
# [-0.39987498918642217, -1.2330292028239747, 1.8821991125689905, -1.876054426232809, 1.4084687232971191, -0.36796313921083623]
#camera1 
# -0.438117806111471, -1.499540166263916, 2.071566406880514, -1.9899307690062464, 2.1313891410827637, -0.3736456076251429
JOINT_START = [-0.026226345692769826, -2.029254738484518, -1.7494915167437952, -1.1875627676593226, -1.5812795797931116, 0.5651097893714905]
if __name__ == '__main__':

    NODE_NAME = 'easy_handeye_mover'

    rospy.init_node(NODE_NAME)

    ur_control = MoveGroupPythonIntefaceTutorial()
    ur_control.group.set_max_velocity_scaling_factor(0.1)
    ur_control.group.set_max_acceleration_scaling_factor(0.2)
    rospy.sleep(1)

    ur_control.remove_allobjects()
    # ur_control.setup_scene()
    # ur_control.group.set_workspace()
    ur_control.group.get_current_joint_values()
    # add_tagbox(ur_control)
    # attach_tagbox(ur_control)

    ur_control.go_to_joint_state(*JOINT_START)

    # # ee_link
    # start_pose = init_pose

    # # tool0_controller
    # # start_pose = gen_pose(0.201706053302, -0.591935787317, 0.317344266852, 0.185197669946, 0.0965404639306, 0.0250100642958, 0.97762787325)
    # current_plan = ur_control.plan_to_pose(start_pose)
    # ur_control.group.execute(current_plan)


    # while rospy.get_time() == 0.0:
    #     pass
    # print "============= !!!!! Attention !!!!!! =============\n" \
    #       "  Are you ready to press the 'Emergency Button' ? \n" \
    #       "=================================================="
    # ipt = raw_input("Continue? y/n: \n")[0]
    # if ipt == 'y':
    #     pass
    # else:
    #     sys.exit(-1)

    # ur_control.group.set_end_effector_link('ee_link')
    curpose = ur_control.group.get_current_pose().pose
    print ('curpose: {}, {}, {}, {}, {}, {}, {}'. \
        format(curpose.position.x,curpose.position.y,curpose.position.z,
               curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w))

    # x,y,z = euler_from_quaternion([curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w])
    # x*=180./np.pi
    # y*=180./np.pi
    # z*=180./np.pi
    # print('curpose: {}, {}, {}, {}, {}, {}, {}'. \
    #       format(curpose.position.x, curpose.position.y, curpose.position.z,
    #              curpose.orientation.x, curpose.orientation.y, curpose.orientation.z, curpose.orientation.w))

    # # 到起始joint状态
    # cm.go_to_joint_state(math.radians(-82), math.radians(-57), math.radians(103), math.radians(-118), math.radians(86), math.radians(-5))
    #
    # # 到起始pose状态
    # start_pose=gen_pose(0.188030545591, -0.595997693534, 0.213596399722, 0.599286810258, -0.536236615971, 0.403848155934, 0.436133325591)
    # cm.start_pose = start_pose
    # current_plan = cm.plan_to_pose(start_pose)
    # cm.execute_plan(current_plan)
    # rospy.sleep(3.0)

    # rotation
    basis = np.eye(3)

    angle_move=math.radians(30)
    angle_delta=[-angle_move, angle_move,
                 -angle_move, angle_move,
                 -angle_move, angle_move]
    # angle_delta[5]=math.radians(30)

    position_delta=[-0.05, 0.05]

    final_rots = []
    for i in range(3):
        final_rots.append(quaternion_from_euler(*basis[i] * angle_delta[i*2]))
        #final_rots.append(quaternion_from_euler(*basis[i] * angle_delta[i*2]/2))
        #final_rots.append(quaternion_from_euler(*basis[i] * angle_delta[i*2+1]/2))
        final_rots.append(quaternion_from_euler(*basis[i] * angle_delta[i*2+1]))

    final_poses = []
    final_poses.append(curpose)
    for rot in final_rots:
        fp = deepcopy(curpose)
        ori = fp.orientation
        combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
        fp.orientation = Quaternion(*combined_rot)
        final_poses.append(fp)

    # translation
    for p in position_delta:
        goal_pose = deepcopy(curpose)
        for i in range(3):
            if i == 0:
                goal_pose.position.x += p
            elif i == 1:
                goal_pose.position.y += p
            else:
                goal_pose.position.z += p

            for rot in final_rots:
                goal_pose_sample = deepcopy(goal_pose)
                ori = goal_pose_sample.orientation
                combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
                goal_pose_sample.orientation = Quaternion(*combined_rot)
                final_poses.append(goal_pose_sample)


    # goal_pose = deepcopy(curpose)
    # goal_pose.position.y+=0.04
    # ori = goal_pose.orientation
    # rot = final_rots[0]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)
    
    

    # goal_pose = deepcopy(curpose)
    # goal_pose.position.y-=0.05
    # ori = goal_pose.orientation
    # rot = final_rots[1]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)


    # goal_pose = deepcopy(curpose)
    # goal_pose.position.x-=0.08
    # ori = goal_pose.orientation
    # rot = final_rots[2]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)

    # goal_pose = deepcopy(curpose)
    # goal_pose.position.x+=0.08
    # ori = goal_pose.orientation
    # rot = final_rots[3]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)

    # goal_pose = deepcopy(curpose)
    # goal_pose.position.z-=0.1
    # ori = goal_pose.orientation
    # rot = final_rots[4]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)

    # goal_pose = deepcopy(curpose)
    # goal_pose.position.z-=0.05
    # ori = goal_pose.orientation
    # rot = final_rots[5]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)

    # goal_pose = deepcopy(curpose)
    # goal_pose.position.x-=0.05
    # ori = goal_pose.orientation
    # rot = final_rots[6]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)

    # goal_pose = deepcopy(curpose)
    # goal_pose.position.x+=0.05
    # ori = goal_pose.orientation
    # rot = final_rots[7]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)

    # goal_pose = deepcopy(curpose)
    # goal_pose.position.y-=0.06
    # ori = goal_pose.orientation
    # rot = final_rots[8]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)

    # goal_pose = deepcopy(curpose)
    # goal_pose.position.y+=0.06
    # ori = goal_pose.orientation
    # rot = final_rots[9]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)

    # goal_pose = deepcopy(curpose)
    # goal_pose.position.z+=0.07
    # ori = goal_pose.orientation
    # rot = final_rots[10]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)

    # goal_pose = deepcopy(curpose)
    # goal_pose.position.z+=0.05
    # ori = goal_pose.orientation
    # rot = final_rots[11]
    # combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    # goal_pose.orientation = Quaternion(*combined_rot)
    # final_poses.append(goal_pose)

    


    # start_pose = gen_pose(-0.137979913015, -0.683912014087, 0.3488927987, 0.612859957908, -0.555459157166, 0.427841060317, 0.364444541489)
    # final_poses.append(start_pose)
    # for rot in final_rots:
    #     fp = deepcopy(curpose)
    #     ori = fp.orientation
    #     combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
    #     fp.orientation = Quaternion(*combined_rot)
    #     final_poses.append(fp)
    

    # goal_pose = deepcopy(curpose)
    # goal_pose.position.z-=0.2
    # final_poses.append(goal_pose)


    # goal_pose=gen_pose(0.216160085878, -0.646681180358, 0.410225377477, 0.631432448921, -0.548988372465, 0.371512297784, 0.402347414485)
    # final_poses.append(goal_pose)

    # HandeyeClient
    client = HandeyeClient()
    sample_list=client.get_sample_list()
    for i in range(len(sample_list.hand_world_samples.transforms)):
        sample_list=client.remove_sample(0)

    for goal_pose in final_poses:
        (current_plan,_) = ur_control.plan_cartesian_path(goal_pose)
        #print(current_plan)
        ur_control.group.execute(current_plan)
        rospy.sleep(3.0)
        sample_list = client.take_sample()

    result = client.compute_calibration()
    transform = result.calibration.transform.transform
    print (transform)

    # analysis-------------------------------------------------
    tf_world2camera = transform_to_mat44(transform)

    sample_list = client.get_sample_list()
    tf_hand2makers=[]
    l_tf_hand2makers = []
    for i in range(len(sample_list.hand_world_samples.transforms)):
        tf_camera2marker = transform_to_mat44(sample_list.camera_marker_samples.transforms[i])
        tf_hand2world = transform_to_mat44(sample_list.hand_world_samples.transforms[i])
        tf_hand2maker = tf_hand2world.dot(tf_world2camera).dot(tf_camera2marker)
        tf_hand2makers.append(tf_hand2maker)
        l_tf_hand2makers.append(mat44_to_xyzrpy(tf_hand2maker))

    pose_array=l_tf_hand2makers
    pose_mean = np.mean(pose_array, axis=0)
    pose_std = np.std(pose_array, axis=0)
    pose_err = pose_array-pose_mean

    err_info = "hand2maker(x, y, z(m), r, p, y(deg)): {}\n" \
               "std:     {}\n" \
               "max err: {}".format(pose_mean,pose_std.tolist(), np.max(np.abs(pose_err), axis=0).tolist())
    print (err_info)

    launch_str = '<!--\n' + err_info + '\n-->' +"""
<launch>
    <node pkg="tf" type="static_transform_publisher" name="base_cam_broadcaster" args="{} {} {} {} {} {} {} base camera0 100" />
</launch>
    """.format(transform.translation.x, transform.translation.y, transform.translation.z,
               transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
    save_file(launch_str, path_base_cam_tf_pub)

    # client.save()
    rospy.loginfo('save file!')
