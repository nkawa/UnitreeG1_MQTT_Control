import math
import json
import time
import numpy as np
from paho.mqtt import client as mqtt


from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from unitree_sdk2py.core.channel import ChannelSubscriber,ChannelPublisher, ChannelFactoryInitialize

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import PressSensorState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_

from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_


DEX3_LEFT_CMD = "rt/dex3/left/cmd"
DEX3_LEFT_STATE = "rt/lf/dex3/left/state"

DEX3_RIGHT_CMD = "rt/dex3/right/cmd"
DEX3_RIGHT_STATE = "rt/lf/dex3/right/state"


OPEN_HAND_RIGHT   = [ -0.031442, 0.061043, -0.033539, 0.100155, 0.024843 ,-0.036757, -0.059524]
CLOSE_HAND_RIGHT  = [ -0.453667 , -0.173300 , -1.1622095, 0.807189  , 1.4322829  , 0.808432 , 1.3445789]

OPEN_HAND_LEFT   = [ -0.031442, 0.061043, -0.033539, 0.100155, 0.024843 ,-0.036757, -0.059524]
CLOSE_HAND_LEFT  = [ -0.514167, -0.327391 , -0.046473, -0.543798 ,-1.05617  , -1.316120 , -0.546621]


close_hand_right = np.array(CLOSE_HAND_RIGHT)
open_hand_right = np.array(OPEN_HAND_RIGHT)
diff_right = close_hand_right - open_hand_right

close_hand_left = np.array(CLOSE_HAND_LEFT)
open_hand_left = np.array(OPEN_HAND_LEFT)
diff_left = close_hand_left - open_hand_left   

def HandStateHandlerLeft(msg: HandState_):
   global screen
   
   ms = msg.motor_state    
   msarray = [ ms[0].q, ms[1].q, ms[2].q, ms[3].q, ms[4].q ,ms[5].q, ms[6].q ]
   screen.addstr(20,2,"  Left Hand s: " + str(list(map(lambda r:int((r*180/math.pi)*1000)/1000, msarray))) + "       ")



class UnitreeG1_Dex3MonitorController:
    monitor_instance = None
    def __init__(self, mqtt_client: mqtt.Client):
        if UnitreeG1_Dex3MonitorController.monitor_instance is not None:
            raise Exception("Only one instance of UnitreeG1_Dex3Monitor is allowed.")
        
        UnitreeG1_Dex3MonitorController.monitor_instance = self
        self.client = mqtt_client

        self.r_curr_time = time.perf_counter()
        self.l_curr_time = time.perf_counter()
        ChannelFactoryInitialize(0, "enp2s0")
        self.pubR = ChannelPublisher(DEX3_RIGHT_CMD, HandCmd_)
        self.pubR.Init()
    
        self.pubL = ChannelPublisher(DEX3_LEFT_CMD, HandCmd_)
        self.pubL.Init()
      
        self.subR = ChannelSubscriber(DEX3_RIGHT_STATE, HandState_)
        self.subR.Init(self.HandStateHandler, 10)
        self.subL = ChannelSubscriber(DEX3_LEFT_STATE, HandState_)
        self.subL.Init(self.HandStateHandlerLeft, 10)
        
        self.rightClosePos = 0  # 0 が　open  1.0 が close
        self.leftClosePos = 0
        self.r_init = True
        self.l_init = True
        self.r_last_time = time.perf_counter()
        self.l_last_time = self.r_last_time

    def HandStateHandler(self, msg: HandState_):
        curr_time = time.perf_counter()
        ms = msg.motor_state
        self.r_msarray = [ ms[0].q, ms[1].q, ms[2].q, ms[3].q, ms[4].q ,ms[5].q, ms[6].q ]
        if curr_time - self.r_last_time > 0.5:
            self.r_last_time = curr_time   
            print("Right: ", str(list(map(lambda r:int((r*180/math.pi)*1000)/1000, self.r_msarray))))

        # 最も近いclosePos を見つけるべし。。。
        # 全部の差分の割合の平均でいいかな。。。        
        if self.r_init: # 初めて受け取った
            self.r_init = False
            rms = np.array(self.r_msarray)
            diff_ratios = (rms - open_hand_right) / diff_right
            self.rightClosePos = np.clip( np.mean(diff_ratios), 0.0, 1.0)
            return
    
    def HandStateHandlerLeft(self, msg: HandState_):
        curr_time = time.perf_counter()
        ms = msg.motor_state
        self.l_msarray = [ ms[0].q, ms[1].q, ms[2].q, ms[3].q, ms[4].q ,ms[5].q, ms[6].q ]
        if curr_time - self.l_last_time > 0.5:
            self.l_last_time = curr_time   
            print("Left : ",str(list(map(lambda r:int((r*180/math.pi)*1000)/1000, self.l_msarray))))        
        if self.l_init: # 初めて受け取った
            self.l_init = False
            rms = np.array(self.l_msarray)
            diff_ratios = (rms - open_hand_left) / diff_left
            self.leftClosePos = np.clip( np.mean(diff_ratios), 0.0, 1.0)
            return

    def send_right_hand_command(self, dir): 
        if self.r_init == False:
            return  # 受信するまでは命令しない

        if dir > 0:
            self.rightClosePos += 0.02
            if self.rightClosePos > 1.0:
                self.rightClosePos = 1.0
        elif dir < 0:
            self.rightClosePos -= 0.05
            if self.rightClosePos < 0.0:
                self.rightClosePos = 0.0
  
        cmd = unitree_hg_msg_dds__HandCmd_()
        q = [0,0,0,0,0,0,0]
        for i in range(7):      
            cmd.motor_cmd[i].q = open_hand_right[i] +  diff_right[i]*self.rightClosePos
            q[i] = float(cmd.motor_cmd[i].q )
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kp = 5
            cmd.motor_cmd[i].kd = 0.1
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].mode = 1  # position mode
            self.pubR.Write(cmd) 
    
    def send_left_hand_command(self, dir): 
        if self.l_init == False:
            return  # 受信するまでは命令しない

        if dir > 0:
            self.leftClosePos += 0.02
            if self.leftClosePos > 1.0:
                self.leftClosePos = 1.0
        elif dir < 0:
            self.leftClosePos -= 0.05
            if self.leftClosePos < 0.0:
                self.leftClosePos = 0.0
  
        cmd = unitree_hg_msg_dds__HandCmd_()
        q = [0,0,0,0,0,0,0]
        for i in range(7):      
            cmd.motor_cmd[i].q = open_hand_left[i] +  diff_left[i]*self.leftClosePos
            q[i] = float(cmd.motor_cmd[i].q )
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kp = 5
            cmd.motor_cmd[i].kd = 0.1
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].mode = 1  # position mode
            self.pubL.Write(cmd)
  
