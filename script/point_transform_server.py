#!/usr/bin/env python
from __future__ import print_function
from ur_control.srv import CameraToWorld, CameraToWorldResponse
import tf2_ros
from ur_control import helper
import rospy
import geometry_msgs.msg
from std_msgs.msg import Int8

global trans_mat

def handle_transform_point(req):
    global trans_mat
    print("receive a request!")
    num = len(req.pc)/3
    pw_ar = []
    for i in range(int(num)):
        pc = geometry_msgs.msg.Point(req.pc[0+i*3], req.pc[1+i*3], req.pc[2+i*3])
        #print(pc)
        pw = helper.transform_point_from_tf(trans_mat, pc)
        #print([pw.x, pw.y, pw.z])
        pw_ar += [pw.x, pw.y, pw.z]
    return CameraToWorldResponse(pw_ar)

def transform_point_server():
    
    s = rospy.Service('camera_to_world', CameraToWorld, handle_transform_point)
    print("Ready to transform point.")
    rospy.spin()


if __name__ == "__main__":
    global trans_mat, trans_mat1

    rospy.init_node('transform_point_server')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.sleep(1)

    ###################################################
    ### get the translation of baselink to camera 0 ###
    ###################################################

    trans = tfBuffer.lookup_transform('base_link', 'camera', rospy.Time())
    # print("transformation: ----------")
    # print(trans)

    translation = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
    rotation = (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)

    trans_mat = helper.transrot_to_mat44(translation, rotation)
    
    ###################################################
    ### get the translation of baselink to camera 1 ###
    ###################################################


    transform_point_server()