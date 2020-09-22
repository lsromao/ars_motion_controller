#!/usr/bin/env python

import numpy as np
from numpy import *

import os


# ROS

import rospy

import tf_conversions as tf


#
import ars_lib_helpers

#
import ars_pid




class ArsMotionController:

  #######

  # References
  #
  flag_set_robot_pose_ref = False
  robot_posi_ref = None
  robot_atti_quat_simp_ref = None
  #
  flag_set_robot_velo_world_ref = False
  robot_velo_lin_world_ref = None
  robot_velo_ang_world_ref = None
  #
  flag_set_robot_velo_cmd_ref = False
  # m/s
  robot_velo_lin_cmd_ref = None
  # rad/s
  robot_velo_ang_cmd_ref = None


  # Feedback
  #
  flag_set_robot_pose = False
  robot_posi = None
  robot_atti_quat_simp = None
  #
  flag_set_robot_vel_world = False
  robot_velo_lin_world = None
  robot_velo_ang_world = None


  # Commands
  robot_velo_cmd_time_stamp = rospy.Time(0.0, 0.0)
  robot_velo_lin_cmd = None
  robot_velo_ang_cmd = None
  

  # Loops Internal

  # Vel loop
  # Not needed!
  #
  #vel_loop_time_stamp_ros = rospy.Time(0.0, 0.0)
  #vel_loop_out_lin_cmd = None
  #vel_loop_out_ang_cmd = None
  
  # Pos loop
  #
  pos_loop_time_stamp_ros = rospy.Time(0.0, 0.0)
  flag_set_pos_loop_out = False
  pos_loop_out_lin_cmd = None
  pos_loop_out_ang_cmd = None


  # PIDs
  # Pos
  pos_x_pid = ars_pid.PID()
  pos_y_pid = ars_pid.PID()
  pos_z_pid = ars_pid.PID()
  att_yaw_pid = ars_pid.PID()

  # Vel
  vel_lin_x_pid = ars_pid.PID()
  vel_lin_y_pid = ars_pid.PID()
  vel_lin_z_pid = ars_pid.PID()
  vel_ang_z_pid = ars_pid.PID()




  #########

  def __init__(self):


    # Commands
    self.robot_velo_cmd_time_stamp = rospy.Time(0.0, 0.0)
    self.robot_velo_lin_cmd = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd = np.zeros((1,), dtype=float)

    # Feedback
    #
    self.flag_set_robot_pose = False
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()
    #
    self.flag_set_robot_vel_world = False
    self.robot_velo_lin_world = np.zeros((3,), dtype=float)
    self.robot_velo_ang_world = np.zeros((1,), dtype=float)

    # References
    #
    self.flag_set_robot_pose_ref = False
    self.robot_posi_ref = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp_ref = ars_lib_helpers.Quaternion.zerosQuatSimp()
    #
    self.flag_set_robot_velo_world_ref = False
    self.robot_velo_lin_world_ref = np.zeros((3,), dtype=float)
    self.robot_velo_ang_world_ref = np.zeros((1,), dtype=float)
    #
    self.flag_set_robot_velo_cmd_ref = False
    self.robot_velo_lin_cmd_ref = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd_ref = np.zeros((1,), dtype=float)

    # Internal
    # Vel loop
    # Not needed!
    #self.vel_loop_time_stamp_ros = rospy.Time(0.0, 0.0)
    #self.vel_loop_out_lin_cmd = np.zeros((3,1), dtype=float)
    #self.vel_loop_out_ang_cmd = np.zeros((1,1), dtype=float)
    # Pos loop
    self.pos_loop_time_stamp_ros = rospy.Time(0.0, 0.0)
    self.flag_set_pos_loop_out = False
    self.pos_loop_out_lin_cmd = np.zeros((3,), dtype=float)
    self.pos_loop_out_ang_cmd = np.zeros((1,), dtype=float)


    # TUNE PID
    # Pos
    #
    self.pos_x_pid.gains['P'] = 1.0
    #
    self.pos_y_pid.gains['P'] = 1.0
    #
    self.pos_z_pid.gains['P'] = 1.0
    #
    self.att_yaw_pid.gains['P'] = 1.0

    # Vel
    #
    self.vel_lin_x_pid.gains['P'] = 1.0
    #
    self.vel_lin_y_pid.gains['P'] = 1.0
    #
    self.vel_lin_z_pid.gains['P'] = 1.0
    #
    self.vel_ang_z_pid.gains['P'] = 1.0


    # End
    return

  def setRobotPosRef(self, robot_posi_ref, robot_atti_quat_simp_ref):

    self.flag_set_robot_pose_ref = True

    self.robot_posi_ref = robot_posi_ref
    self.robot_atti_quat_simp_ref = robot_atti_quat_simp_ref

    return

  def setRobotVelWorldRef(self, lin_vel_world_ref, ang_vel_world_ref):

    self.flag_set_robot_velo_world_ref = True

    self.robot_velo_lin_world_ref = lin_vel_world_ref
    self.robot_velo_ang_world_ref = ang_vel_world_ref

    return

  def setRobotVelCmdRef(self, lin_vel_cmd_ref, ang_vel_cmd_ref):

    self.flag_set_robot_velo_cmd_ref = True

    self.robot_velo_lin_cmd_ref = lin_vel_cmd_ref
    self.robot_velo_ang_cmd_ref = ang_vel_cmd_ref

    return

  def setRobotPose(self, robot_posi, robot_atti_quat_simp):
    
    self.flag_set_robot_pose = True

    self.robot_posi = robot_posi
    self.robot_atti_quat_simp = robot_atti_quat_simp

    return

  def setRobotVelWorld(self, lin_vel_world, ang_vel_world):

    self.flag_set_robot_vel_world = True

    self.robot_velo_lin_world = lin_vel_world
    self.robot_velo_ang_world = ang_vel_world

    return

  def getRobotVeloCmdTimeStamp(self):
    return self.robot_velo_cmd_time_stamp

  def getRobotVeloLinCmd(self):
    return self.robot_velo_lin_cmd

  def getRobotVeloAngCmd(self):
    return self.robot_velo_ang_cmd


  def velLoopMotionController(self, time_stamp_ros):

    # Time stamp
    self.robot_velo_cmd_time_stamp = time_stamp_ros

    # Conversion
    robot_velo_lin_robot = ars_lib_helpers.Conversions.convertVelLinFromWorldToRobot(self.robot_velo_lin_world, self.robot_atti_quat_simp)
    robot_velo_ang_robot = ars_lib_helpers.Conversions.convertVelAngFromWorldToRobot(self.robot_velo_ang_world, self.robot_atti_quat_simp)


    # Linear: x & y
    # TODO BY STUDENT: BEGINNING


    self.robot_velo_lin_cmd[0] = 0.0
    self.robot_velo_lin_cmd[1] = 0.0
    

    # TODO BY STUDENT: END


    # Linear: z
    if(self.flag_set_pos_loop_out & self.flag_set_robot_vel_world):
      error_vel_lin_z = self.pos_loop_out_lin_cmd[2] - robot_velo_lin_robot[2]
    else:
      error_vel_lin_z = 0.0
    self.robot_velo_lin_cmd[2] = self.robot_velo_lin_cmd_ref[2] + self.vel_lin_z_pid.call(time_stamp_ros, error_vel_lin_z)
    
    # Angular: z
    if(self.flag_set_pos_loop_out & self.flag_set_robot_vel_world):
      error_vel_ang_z = self.pos_loop_out_ang_cmd - robot_velo_ang_robot
    else:
      error_vel_ang_z = 0.0
    self.robot_velo_ang_cmd[0] = self.robot_velo_ang_cmd_ref[0] + self.vel_ang_z_pid.call(time_stamp_ros, error_vel_ang_z)

    # End
    return


  def posLoopMotionController(self, time_stamp_ros):

    # Time stamp
    self.pos_loop_time_stamp_ros = time_stamp_ros

    # Linear: x & y
    # TODO BY STUDENT: BEGINNING


    self.pos_loop_out_lin_cmd[0] = 0.0
    self.pos_loop_out_lin_cmd[1] = 0.0


    # TODO BY STUDENT: END


    # Linear: z
    if(self.flag_set_robot_pose and self.flag_set_robot_pose_ref):
      error_pos_z = self.robot_posi_ref[2] - self.robot_posi[2]
    else:
      error_pos_z = 0.0
    self.pos_loop_out_lin_cmd[2] = self.robot_velo_lin_world_ref[2] + self.pos_z_pid.call(time_stamp_ros, error_pos_z)


    # Angular: z
    if(self.flag_set_robot_pose and self.flag_set_robot_pose_ref):
      error_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()
      error_quat_simp[0] = self.robot_atti_quat_simp_ref[0]*self.robot_atti_quat_simp[0]+self.robot_atti_quat_simp_ref[1]*self.robot_atti_quat_simp[1]
      error_quat_simp[1] = self.robot_atti_quat_simp_ref[1]*self.robot_atti_quat_simp[0]-self.robot_atti_quat_simp_ref[0]*self.robot_atti_quat_simp[1]
      #error_att_z = 2.0 * math.atan(error_quat_simp[1]/error_quat_simp[0])
      error_att_z = 2.0 * error_quat_simp[1]
    else:
      error_att_z = 0.0
    self.pos_loop_out_ang_cmd[0] = self.robot_velo_ang_world_ref[0] + self.att_yaw_pid.call(time_stamp_ros, error_att_z)


    # Flag
    self.flag_set_pos_loop_out = True

    # End
    return
