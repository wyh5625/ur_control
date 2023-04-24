#!/usr/bin/env python
from __future__ import print_function
from ur_control.srv import CameraToWorld, CameraToWorldResponse
import tf2_ros
from ur_control import helper
import rospy
import geometry_msgs.msg
from std_msgs.msg import Int8

global trans_mat0, trans_mat1, cameraInUse

def handle_transform_point(req):
    global trans_mat0, trans_mat1, cameraInUse
    print("receive a request!")
    num = len(req.pc)/3
    pw_ar = []
    for i in range(int(num)):
        pc = geometry_msgs.msg.Point(req.pc[0+i*3], req.pc[1+i*3], req.pc[2+i*3])
        #print(pc)
        if cameraInUse == 0:
            pw = helper.transform_point_from_tf(trans_mat0, pc)
        else:
            pw = helper.transform_point_from_tf(trans_mat1, pc)
        #print([pw.x, pw.y, pw.z])
        pw_ar += [pw.x, pw.y, pw.z]
    return CameraToWorldResponse(pw_ar)

def transform_point_server():
    
    s = rospy.Service('camera_to_world', CameraToWorld, handle_transform_point)
    print("Ready to transform point.")
    rospy.spin()

def camera_callback(msg):
    global cameraInUse
    cameraInUse = msg.data

if __name__ == "__main__":
    global trans_mat0, trans_mat1

    rospy.init_node('transform_point_server')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.sleep(1)

    ###################################################
    ### get the translation of baselink to camera 0 ###
    ###################################################

    trans0 = tfBuffer.lookup_transform('base_link', 'camera0', rospy.Time())
    # print("transformation: ----------")
    # print(trans0)

    translation = (trans0.transform.translation.x, trans0.transform.translation.y, trans0.transform.translation.z)
    rotation = (trans0.transform.rotation.x, trans0.transform.rotation.y, trans0.transform.rotation.z, trans0.transform.rotation.w)

    trans_mat0 = helper.transrot_to_mat44(translation, rotation)
    
    ###################################################
    ### get the translation of baselink to camera 1 ###
    ###################################################

    trans1 = tfBuffer.lookup_transform('base_link', 'camera1', rospy.Time())
    # print("transformation: ----------")
    # print(trans0)

    translation = (trans1.transform.translation.x, trans1.transform.translation.y, trans1.transform.translation.z)
    rotation = (trans1.transform.rotation.x, trans1.transform.rotation.y, trans1.transform.rotation.z, trans1.transform.rotation.w)

    trans_mat1 = helper.transrot_to_mat44(translation, rotation)

    rospy.Subscriber("/camera_in_use", Int8, camera_callback)

    transform_point_server()