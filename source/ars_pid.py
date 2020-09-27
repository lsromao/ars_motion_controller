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

  gains = {'P': 1.0}


  def __init__(self):

    self.prev_time_stamp_ros = rospy.Time(0.0, 0.0)

    self.gains = {'P': 1.0}

    self.control_cmd = 0.0

    return

  def call(self, curr_time_stamp, error):

    #
    control_cmd = 0.0

    # K
    control_cmd = self.gains['P']*error;


    # End
    return control_cmd

