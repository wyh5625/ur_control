#!/usr/bin/env python
import rospy
import tf2_ros
from std_msgs.msg import Float32MultiArray
from ur_control.srv import *
import sys
import tf
import geometry_msgs

# Centroid Marker is used to record the global position of the marker
class CentroidMarker(object):

    def __init__(self, base_frame, tag, offset):
        self.tracking_base_frame = base_frame
        self.tracking_marker_frame = tag
        self.offset = offset

        # tf structures
        # self.tfBuffer = tf2_ros.Buffer()
        # self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        # self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        # self.samples = []
        self.listener = tf.TransformListener()

    def _wait_for_tf(self):
        self.listener.waitForTransform(self.tracking_base_frame, self.tracking_marker_frame, rospy.Time(), rospy.Duration(4.0))

    # def _get_transform(self):
    #     opt = self.tfBuffer.lookup_transform(self.tracking_base_frame, self.tracking_marker_frame, rospy.Time(0),
    #                                          rospy.Duration(5))
    #     return opt
    
    def find_marker(self):
        print("find frame: " + self.tracking_marker_frame)
        self._wait_for_tf()
        print("found frame: " + self.tracking_marker_frame)
        marker_point = geometry_msgs.msg.PointStamped()
        marker_point.header.frame_id = self.tracking_marker_frame
        marker_point.header.stamp = rospy.Time()
        marker_point.point.x = self.offset[0]
        marker_point.point.y = self.offset[1]
        marker_point.point.z = self.offset[2]
        global_point = self.listener.transformPoint(self.tracking_base_frame, marker_point)

        gp = [global_point.point.x, global_point.point.y, global_point.point.z]
        return gp

    # def take_sample(self):
    #     rospy.loginfo("Taking a sample...")
    #     transforms = self._get_transforms()
    #     self.samples.append(transforms)

    # def get_translation(self):
    #     transform = self._get_transform()
    #     translation = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
    #     return translation

    # def get_world_coordinate(self):
    #     trans = self.get_translation()
    #     res = self.camera_to_world(trans)
    #     return res.pw



def centroid_publisher(centroid_maker):
    mc_pub = rospy.Publisher('marker_centroid', Float32MultiArray, queue_size = 10)
    rate = rospy.Rate(10)   # 10hz
    while not rospy.is_shutdown():
        centroid_msg = Float32MultiArray()
        # append coordinates of each marker on the list
        for cm in centroid_maker:
            centroid_msg.data = centroid_msg.data + cm.find_marker()
        mc_pub.publish(centroid_msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('publish_marker', anonymous=True)

    # camera_name = rospy.get_param('~camera_name', 'camera0')
    # tag = rospy.get_param('~tag', 'tag_3')

    # position in marker frame
    offset = [-0.034,0.034,0]
    marker_list = []
    world_frame = "base_link"
    marker_list.append(CentroidMarker(world_frame, "tag_0", offset))
    marker_list.append(CentroidMarker(world_frame, "tag_1", offset))
    marker_list.append(CentroidMarker(world_frame, "tag_2", offset))
    marker_list.append(CentroidMarker(world_frame, "tag_3", offset))
    marker_list.append(CentroidMarker(world_frame, "tag_4", offset))
    #marker_list.append(CentroidMarker(world_frame, "tag_5", offset))

    # publish the position of the marker
    centroid_publisher(marker_list)