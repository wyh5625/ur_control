#!/usr/bin/env python
import rospy
import tf2_ros
from ur_control import helper
from std_msgs.msg import Float64MultiArray
import geometry_msgs.msg

import numpy as np

global pub, trans_mat

def callback(center_point):
    global pub, trans_mat

    out_data = Float64MultiArray()
    for i in range(int(len(center_point.data)/3)):

        p1 = geometry_msgs.msg.Point(center_point.data[0 + i*3], center_point.data[1 + i*3], center_point.data[2 + i*3])
        p11 = geometry_msgs.msg.Point()
        if(p1.x == 0 and p1.y == 0 and p1.z == 0):
            p11.x = 0
            p11.y = 0
            p11.z = 0
        else:
            p11 = helper.transform_point_from_tf(trans_mat, p1)

        out_data.data.extend([p11.x, p11.y, p11.z])

    pub.publish(out_data)




if __name__ == '__main__':
    global pub, trans_mat

    pub = rospy.Publisher('/centroid_in_world_frame', Float64MultiArray, queue_size=10)
    rospy.init_node('center_converter', anonymous=True)
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.sleep(1)

    trans = tfBuffer.lookup_transform('base_link', 'camera', rospy.Time())
    print("transformation: ----------")
    print(trans)

    translation = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
    rotation = (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)

    
    trans_mat = helper.transrot_to_mat44(translation, rotation)
    
    

    rospy.Subscriber("/soft_object_tracking/centroid", Float64MultiArray, callback)


    rospy.spin()