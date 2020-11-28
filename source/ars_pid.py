#!/usr/bin/env python

import numpy as np
from numpy import *

import os

# ROS

import rospy

import tf_conversions as tf

#
import ars_lib_helpers

class PID:
    #######
    prev_time_stamp_ros = rospy.Time(0.0, 0.0)

    gains = {'P': 1.0, 'D': 1.0, 'I': 1.0}

    def __init__(self):
        self.prev_time_stamp_ros = rospy.Time(0.0, 0.0)

        self.gains = {'P': 1.5, 'D': 0.5, 'I': 0.001}

        self.control_cmd = 0.0
        self.prev_error = 0
        self.sum_error = 0

        self.p = 0
        self.i = 0
        self.d = 0

        return

    def call(self, curr_time_stamp, error):
        #
        control_cmd = 0.0

        dt = curr_time_stamp - self.prev_time_stamp_ros
        de = error - self.prev_error
        # K

        self.p = error * self.gains['P']

        self.i = (self.sum_error * dt.to_sec()) * self.gains['I']

        self.d = 0
        if dt.to_sec() > 0:
            self.d = de / dt.to_sec()

        self.prev_time_stamp_ros = curr_time_stamp
        self.prev_error = error
        self.sum_error += error


        control_cmd += self.p  + self.i + (self.d * self.gains['D'])

        # End
        return control_cmd
