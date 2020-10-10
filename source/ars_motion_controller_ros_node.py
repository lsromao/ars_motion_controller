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

    pos_controller_flag = True
    vel_controller_flag = False

    try:
        print """
    p: Enable/Disable Position Controller
    v: Enable/Disable Velocity Controller
    s: Stop
    """
        while (1):
            key = getKey(None, settings)
            if key == 'p':
                if pos_controller_flag:
                    pos_controller_flag = False
                    print('Position Controller Disabled')
                else:
                    pos_controller_flag = True
                    print('Position Controller Enabled')

                ars_motion_controller_ros.controller_position(disable_pos=pos_controller_flag)
            elif key == 'v':
                if vel_controller_flag:
                    vel_controller_flag = False
                    print('Velocity Controller Disabled')
                else:
                    vel_controller_flag = True
                    print('Velocity Controller Enabled')

                ars_motion_controller_ros.controller_velocity(disable_vel=vel_controller_flag)
            else:
                if key == 's':
                    break
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return 0


''' MAIN '''
if __name__ == '__main__':
    main()
