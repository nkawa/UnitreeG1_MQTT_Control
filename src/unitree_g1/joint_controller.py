import time

import numpy as np
import math

from unitree_sdk2py.core.channel import ChannelSubscriber,ChannelPublisher, ChannelFactoryInitialize

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from .joint_index import G1JointIndex
from .joint_monitor import UnitreeG1_JointMonitor


## マルチプロセス対応版にすべき？
class UnitreeG1_JointController:
  controller_instance = None
  def __init__(self):
      if UnitreeG1_JointController.controller_instance is not None:
        raise Exception("Only one instance of UnigreeG1_JointController is allowed.")        
      UnitreeG1_JointController.controller_instance = self
 #     ChannelFactoryInitialize(0, "enp2s0")
      self.left_savetime = 0
      self.saved_left_command = None
#      self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)  
      self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)  
#      self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)  
      self.pub.Init()

      self.waist_yaw = 0 # 腰の角度
      self.low_cmd = unitree_hg_msg_dds__LowCmd_()
#      self.low_cmd = LowCmd_([0, 0], 0, 0, [0, 0], [0, 0], 0, [unitree_go_msg_dds__MotorCmd_() for i in range(20)],
#                unitree_go_msg_dds__BmsCmd_(),
#                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0], 0, 0, 0)
      self.crc = CRC()
      self.last_sent = None
      while UnitreeG1_JointMonitor.monitor_instance is None:        
        print("Waiting for JointMonitor instance...")
        time.sleep(0.5)

      self.mon = UnitreeG1_JointMonitor.monitor_instance
      
      self.arm_joints = [
          G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
          G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
          G1JointIndex.LeftWristRoll,      G1JointIndex.LeftWristPitch,
          G1JointIndex.LeftWristYaw,
          G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
          G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
          G1JointIndex.RightWristRoll,     G1JointIndex.RightWristPitch,
          G1JointIndex.RightWristYaw,
          G1JointIndex.WaistYaw,
          G1JointIndex.WaistRoll,
          G1JointIndex.WaistPitch
      ]
  
  def send_right_arm_command(self, right): 
      # まえの時間との差分
      self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1 # 1:Enable arm_sdk, 0:D
#      np_right = np.deg2rad(np.array(right)) # 
      np_right = np.array(right)
      #まずは右手だけ
      if self.mon.right is None:
        print("No monitor data yet.")
        return
      max_diff = np.abs(np_right - self.mon.right ).max() 
      
      if max_diff > 25.0/180*math.pi:
        print("Detected large right diff in arm command:", max_diff, np_right, self.mon.right)
        return
  
      left = None    
      if time.perf_counter()- self.left_savetime < 0.1:  # under 100msec
        np_left = np.array(self.saved_left_command)

        max_diff = np.abs(np_left - self.mon.left ).max() 
        if max_diff > 25.0/180*math.pi:
          print("Detected large left diff in arm command:", max_diff, np_left, self.mon.left)
          return
        left = self.saved_left_command
           
      #for debug
#      print("SR:", right)
#      return
      
      for i, joint in enumerate(self.arm_joints):
        mcmd = self.low_cmd.motor_cmd[joint]
        if joint >= G1JointIndex.RightShoulderPitch and joint <= G1JointIndex.RightWristYaw:          
          mcmd.q = right[joint-G1JointIndex.RightShoulderPitch]
          mcmd.dq = 0.
          mcmd.tau = 0.
          mcmd.kp = 60.
          mcmd.kd = 1.5
          mcmd.tau_ff = 0.
        elif left != None and joint >= G1JointIndex.LeftShoulderPitch and joint <= G1JointIndex.LeftWristYaw:          
          self.low_cmd.motor_cmd[joint].q = left[joint-G1JointIndex.LeftShoulderPitch]
          mcmd.q = left[joint-G1JointIndex.LeftShoulderPitch]
          mcmd.tau = 0.
          mcmd.kp = 60.
          mcmd.dq = 0.
          mcmd.kd = 1.5
          mcmd.tau_ff = 0.
        else:
          lsms = self.mon.low_state.motor_state[joint]
          if joint == G1JointIndex.WaistYaw:
            mcmd.q = self.waist_yaw
          else:
            mcmd.q = 0
          mcmd.tau = 0.
          mcmd.kp  = 60.
          mcmd.dq  = lsms.dq
          mcmd.kd  = 1.5
          mcmd.tau_ff = 0.
    
      self.low_cmd.crc = self.crc.Crc(self.low_cmd)
# for sport debug
      self.pub.Write(self.low_cmd)
      
  def save_left_arm_command(self, left):
      self.saved_left_command = left
      self.left_savetime = time.perf_counter()
    
  
  def reset_right_arm(self):
    print("Reset right!")
    target = [-0.27433091402053833, -0.12286414206027985, -0.03733086213469505, 0.26636138558387756, -0.037870150059461594, -0.32020315527915955, 0.23122461140155792] 

    self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1 # 1:Enable arm_sdk, 0:D
    np_right = np.array(target)
    
    diff = np_right - self.mon.right 
    div = 2

    #角度変化の最大値を 10度以下にする分割数
    while np.abs(diff/div).max() > 10/180*math.pi:
      div += 1
      if div > 30:
        break

    left = self.saved_left_command

    for ita in range(div):           
      for i, joint in enumerate(self.arm_joints):
        mcmd = self.low_cmd.motor_cmd[joint]
        if joint >= G1JointIndex.RightShoulderPitch and joint <= G1JointIndex.RightWristYaw:         
          index =  joint-G1JointIndex.RightShoulderPitch
          mcmd.q = self.mon.right[index] + (diff[index]/div * (ita+1))
          mcmd.dq = 0.
          mcmd.tau = 0.
          mcmd.kp = 60.
          mcmd.kd = 1.5
          mcmd.tau_ff = 0.
        elif left != None and joint >= G1JointIndex.LeftShoulderPitch and joint <= G1JointIndex.LeftWristYaw:          
          self.low_cmd.motor_cmd[joint].q = left[joint-G1JointIndex.LeftShoulderPitch]
          mcmd.q = left[joint-G1JointIndex.LeftShoulderPitch]
          mcmd.tau = 0.
          mcmd.kp = 60.
          mcmd.dq = 0.
          mcmd.kd = 1.5
          mcmd.tau_ff = 0.
        else:
          lsms = self.mon.low_state.motor_state[joint]
          mcmd.q = lsms.q
          mcmd.tau = 0.
          mcmd.kp  = 60.
          mcmd.dq  = lsms.dq
          mcmd.kd  = 1.5
          mcmd.tau_ff = 0.
    
      self.low_cmd.crc = self.crc.Crc(self.low_cmd)
      self.pub.Write(self.low_cmd)
      time.sleep(0.1)


  def reset_left_arm(self):
    print("Reset right!")
 
    target =   [-0.10218948870897293, 0.1596677154302597, -0.04013517126441002, -0.01477654930204153, -0.08196011930704117, -0.0996243879199028, -0.11140535771846771]

    self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1 # 1:Enable arm_sdk, 0:D
    np_left = np.array(target)
    
    diff = np_left - self.mon.left
    div = 2

    #角度変化の最大値を 10度以下にする分割数
    while np.abs(diff/div).max() > 10/180*math.pi:
      div += 1
      if div > 30:
        break

    right = self.mon.right

    for ita in range(div):           
      for i, joint in enumerate(self.arm_joints):
        mcmd = self.low_cmd.motor_cmd[joint]
        if joint >= G1JointIndex.RightShoulderPitch and joint <= G1JointIndex.RightWristYaw:         
          index =  joint-G1JointIndex.RightShoulderPitch
          mcmd.q = right[index] 
          mcmd.dq = 0.
          mcmd.tau = 0.
          mcmd.kp = 60.
          mcmd.kd = 1.5
          mcmd.tau_ff = 0.
        elif left != None and joint >= G1JointIndex.LeftShoulderPitch and joint <= G1JointIndex.LeftWristYaw:          
          index =  joint-G1JointIndex.LeftShoulderPitch
          mcmd.q = self.mon.left[index]+ (diff[index]/div *(ita+1))
          mcmd.tau = 0.
          mcmd.kp = 60.
          mcmd.dq = 0.
          mcmd.kd = 1.5
          mcmd.tau_ff = 0.
        else:
          lsms = self.mon.low_state.motor_state[joint]
          mcmd.q = lsms.q
          mcmd.tau = 0.
          mcmd.kp  = 60.
          mcmd.dq  = lsms.dq
          mcmd.kd  = 1.5
          mcmd.tau_ff = 0.
    
      self.low_cmd.crc = self.crc.Crc(self.low_cmd)
      self.pub.Write(self.low_cmd)
      time.sleep(0.1)
      
  def turn_waist(self, dir):
    self.waist_yaw += dir
    


