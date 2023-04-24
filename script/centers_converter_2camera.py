#!/usr/bin/env python
import rospy
import tf2_ros
from ur_control import helper
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int8
import geometry_msgs.msg

import numpy as np

global pub, trans_mat0, trans_mat1, cameraInUse

def tf_callback(center_point):
    global pub, trans_mat0, trans_mat1, cameraInUse

    out_data = Float64MultiArray()
    for i in range(int(len(center_point.data)/3)):

        p1 = geometry_msgs.msg.Point(center_point.data[0 + i*3], center_point.data[1 + i*3], center_point.data[2 + i*3])
        p11 = geometry_msgs.msg.Point()
        if(p1.x == 0 and p1.y == 0 and p1.z == 0):
            p11.x = 0
            p11.y = 0
            p11.z = 0
        else:
            if (cameraInUse == 0):
                p11 = helper.transform_point_from_tf(trans_mat0, p1)
            else:
                p11 = helper.transform_point_from_tf(trans_mat1, p1)

        out_data.data.extend([p11.x, p11.y, p11.z])

    pub.publish(out_data)

def camera_callback(msg):
    global cameraInUse
    cameraInUse = msg.data


if __name__ == '__main__':
    global pub, trans_mat, cameraInUse

    # the default camera is 0
    cameraInUse = 0

    pub = rospy.Publisher('/centroid_in_world_frame', Float64MultiArray, queue_size=10)
    rospy.init_node('center_converter', anonymous=True)
    
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

    

    rospy.Subscriber("/soft_object_tracking/centroid", Float64MultiArray, tf_callback)
    rospy.Subscriber("/camera_in_use", Int8, camera_callback)


    rospy.spin()