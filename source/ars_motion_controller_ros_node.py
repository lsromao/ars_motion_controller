#!/usr/bin/env python

import numpy as np
from numpy import *

import os


import rospy


from ars_motion_controller_ros import *
import sys, select, termios, tty


def getKey(key_timeout, settings):
  tty.setraw(sys.stdin.fileno())
  rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
  if rlist:
    key = sys.stdin.read(1)
  else:
    key = ''
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key


def main():

  settings = termios.tcgetattr(sys.stdin)
  ars_motion_controller_ros = ArsMotionControllerRos()

  ars_motion_controller_ros.init()
  ars_motion_controller_ros.open()

  try:
    print """
    p: Enable Position Controller 
    o: Disable Position Controller
    v: Enable Velocity Controller
    c: Disable Velocity Controller
    s: Stop 
    """

    while (1):
      key = getKey(None, settings)
      if key == 'p':
        print('Position Controller Enabled')
        ars_motion_controller_ros.controller_position(disable_pos=True)
      elif key == 'o':
        print('Position Controller Disabled')
        ars_motion_controller_ros.controller_position(disable_pos=False)
      elif key == 'v':
        print('Velocity Controller Enabled')
        ars_motion_controller_ros.controller_velocity(disable_vel=True)
      elif key == 'c':
        print('Velocity Controller Disabled')
        ars_motion_controller_ros.controller_velocity(disable_vel=False)
      else:
        if (key == 's'):
          break
  except rospy.ROSInterruptException:
    pass
  finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return 0



''' MAIN '''
if __name__ == '__main__':

  main()