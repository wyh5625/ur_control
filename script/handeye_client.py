#!/usr/bin/env python

import rospy
import std_srvs
from std_srvs import srv
import easy_handeye as hec
from easy_handeye import srv


class HandeyeClient(object):

    def __init__(self):
        ns = '/easy_handeye_eye_on_base/'

        rospy.wait_for_service(ns+hec.GET_SAMPLE_LIST_TOPIC)
        self.get_sample_proxy = rospy.ServiceProxy(ns+hec.GET_SAMPLE_LIST_TOPIC, hec.srv.TakeSample)
        rospy.wait_for_service(ns+hec.TAKE_SAMPLE_TOPIC)
        self.take_sample_proxy = rospy.ServiceProxy(ns+hec.TAKE_SAMPLE_TOPIC, hec.srv.TakeSample)
        rospy.wait_for_service(ns+hec.REMOVE_SAMPLE_TOPIC)
        self.remove_sample_proxy = rospy.ServiceProxy(ns+hec.REMOVE_SAMPLE_TOPIC, hec.srv.RemoveSample)
        rospy.wait_for_service(ns+hec.COMPUTE_CALIBRATION_TOPIC)
        self.compute_calibration_proxy = rospy.ServiceProxy(ns+hec.COMPUTE_CALIBRATION_TOPIC, hec.srv.ComputeCalibration)
        rospy.wait_for_service(ns+hec.SAVE_CALIBRATION_TOPIC)
        self.save_calibration_proxy = rospy.ServiceProxy(ns+hec.SAVE_CALIBRATION_TOPIC, std_srvs.srv.Empty)

    def get_sample_list(self):
        return self.get_sample_proxy().samples

    def take_sample(self):
        return self.take_sample_proxy().samples

    def remove_sample(self, index):
        return self.remove_sample_proxy(hec.srv.RemoveSampleRequest(sample_index=index)).samples

    def compute_calibration(self):
        return self.compute_calibration_proxy()

    def save(self):
        return self.save_calibration_proxy()
