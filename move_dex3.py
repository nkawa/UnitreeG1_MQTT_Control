import time
import math

import curses

import numpy as np

from unitree_sdk2py.core.channel import ChannelSubscriber,ChannelPublisher, ChannelFactoryInitialize

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import PressSensorState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
#from unitree_sdk2py.idl.unitree_hg.msg.dds_ import PressureSensorState_

from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

#uni = UnitreeG1_MQTT()

DEX3_LEFT_CMD = "rt/dex3/left/cmd"
DEX3_LEFT_STATE = "rt/lf/dex3/left/state"

DEX3_RIGHT_CMD = "rt/dex3/right/cmd"
DEX3_RIGHT_STATE = "rt/lf/dex3/right/state"
#DEX3_RIGHT_STATE = "rt/dex3/right/state"

MAX_LIMITS_LEFT =  [  1.05 ,  1.05  , 1.75 ,   0   ,  0    , 0     , 0   ] # // set max motor value
MIN_LIMITS_LEFT =  [ -1.05 , -0.724 ,   0  , -1.57 , -1.75 , -1.57  ,-1.75 ]
#MAX_LIMITS_RIGHT = {  1.05 , 0.742  ,   0  ,  1.57 , 1.75  , 1.57  , 1.75};
#MIN_LIMITS_RIGHT = { -1.05 , -1.05  , -1.75,    0  ,  0    ,   0   ,0    };


OPEN_HAND_RIGHT   = [ -0.031442, 0.061043, -0.033539, 0.100155, 0.024843 ,-0.036757, -0.059524]
CLOSE_HAND_RIGHT  = [ -0.453667 , -0.173300 , -1.1622095, 0.807189  , 1.4322829  , 0.808432 , 1.3445789]

OPEN_HAND_LEFT   = [ -0.031442, 0.061043, -0.033539, 0.100155, 0.024843 ,-0.036757, -0.059524]
CLOSE_HAND_LEFT  = [ -0.514167, -0.327391 , -0.046473, -0.543798 ,-1.05617  , -1.316120 , -0.546621]


last_time = time.perf_counter()

## // level によって、グリップを変える
## モードの概念が必要か？

# 本当は初期状態を取得すべき？

rightHand = 0  # 0 が　open  1.0 が close
leftHand = 0  # 0 が　open  1.0 が close


close_hand_right = np.array(CLOSE_HAND_RIGHT)
open_hand_right = np.array(OPEN_HAND_RIGHT)
diff = close_hand_right - open_hand_right

close_hand_left = np.array(CLOSE_HAND_LEFT)
open_hand_left = np.array(OPEN_HAND_LEFT)
diff_left = close_hand_left - open_hand_left

screen = None

def HandStateHandler(msg: HandState_):
   global last_time
   global screen
   
   curr_time = time.perf_counter()
#   print("Diff Time:", int((curr_time - last_time)*1000000)/1000, "ms")  
   last_time = curr_time
#        print("Wireless:", msg.wireless_remote)
#   print("p_v,p_a, s_v, dev_d:", msg.power_v, msg.power_a, msg.system_v, msg.device_v)

    #        print("Motor len:", len(msg.motor_state))
   ms = msg.motor_state
    
   msarray = [ ms[0].q, ms[1].q, ms[2].q, ms[3].q, ms[4].q ,ms[5].q, ms[6].q ]
#    print("Motor:", msg.motor_state)
   screen.addstr(14,2," Right Hand s: " + str(list(map(lambda r:int((r*180/math.pi)*1000)/1000, msarray))) + "       ")

#   print("Right:", list(map(lambda r:int((r*180/math.pi)*1000)/1000, msarray)))   
#   print("RightDeg:",  msarray)  
#   print("Mode", ms[0].mode, ms[1].mode, ms[2].mode, ms[3].mode, ms[4].mode ,ms[5].mode, ms[6].mode )
def HandStateHandlerLeft(msg: HandState_):
   global screen
   
   ms = msg.motor_state    
   msarray = [ ms[0].q, ms[1].q, ms[2].q, ms[3].q, ms[4].q ,ms[5].q, ms[6].q ]
   screen.addstr(20,2,"  Left Hand s: " + str(list(map(lambda r:int((r*180/math.pi)*1000)/1000, msarray))) + "       ")


def send_right_hand_command( dir): 
  global rightHand, close_hand_right, open_hand_right, pubR, diff
  if dir > 0:
      rightHand += 0.02
      if rightHand > 1.0:
          rightHand = 1.0
  elif dir < 0:
      rightHand -= 0.05
      if rightHand < 0.0:
          rightHand = 0.0
  
  cmd = unitree_hg_msg_dds__HandCmd_()
  q = [0,0,0,0,0,0,0]
  for i in range(7):      

      cmd.motor_cmd[i].q = open_hand_right[i] +  diff[i]*rightHand
      q[i] = float(open_hand_right[i] +  diff[i]*rightHand)
      cmd.motor_cmd[i].dq = 0.0
      cmd.motor_cmd[i].kp = 5
      cmd.motor_cmd[i].kd = 0.1
      cmd.motor_cmd[i].tau = 0.0
      cmd.motor_cmd[i].mode = 1  # position mode
        
  pubR.Write(cmd) 
  return q
  
def send_left_hand_command( dir):
  global leftHand, close_hand_left, open_hand_left, pubL, diff_left
  if dir > 0:
      leftHand += 0.02
      if leftHand > 1.0:
          leftHand = 1.0
  elif dir < 0:
      leftHand -= 0.05
      if leftHand < 0.0:
          leftHand = 0.0
  
  cmd = unitree_hg_msg_dds__HandCmd_()
  q = [0,0,0,0,0,0,0]
  for i in range(7):      
      cmd.motor_cmd[i].q = open_hand_left[i] +  diff_left[i]*leftHand
      q[i] = float(open_hand_left[i] +  diff_left[i]*leftHand)
      cmd.motor_cmd[i].dq = 0.0
      cmd.motor_cmd[i].kp = 5
      cmd.motor_cmd[i].kd = 0.1
      cmd.motor_cmd[i].tau = 0.0
      cmd.motor_cmd[i].mode = 1  # position mode
        
  pubL.Write(cmd) 
  return q

def main(stdscr):
  global rightHand
  global screen
  screen = stdscr
  stdscr.nodelay(True)
  stdscr.addstr(0,0, "Press o: Open hand, c: Close hand, q: Quit")
  stdscr.refresh()
  q = [0,0,0,0,0,0,0]
  ql = [0,0,0,0,0,0,0]
  
  subR = ChannelSubscriber(DEX3_RIGHT_STATE, HandState_)
  subR.Init(HandStateHandler, 10)
  subL = ChannelSubscriber(DEX3_LEFT_STATE, HandState_)
  subL.Init(HandStateHandlerLeft, 10)

  while True:
    c = stdscr.getch()
    if c == ord('q'):
        break
    elif c == ord('o'):
#        print("Open hand")
        q = send_right_hand_command(-1)
    elif c == ord('c'):
#        print("Close hand")
        q = send_right_hand_command(1)
    elif c == ord('w'):
#        print("Open hand")
        ql = send_left_hand_command(-1)
    elif c == ord('e'):
#        print("Close hand")
        ql = send_left_hand_command(1)
    stdscr.addstr(10,2, " Right hand level: " + str(int((1-rightHand)*100)) + "%   ")
    stdscr.addstr(12,2, " Right hand q: " + str(list(map(lambda r:int((r*180/math.pi)*1000)/1000, q))) + "       ")

    stdscr.addstr(16,2, " Left  hand level: " + str(int((1-leftHand)*100)) + "%   ")
    stdscr.addstr(18,2, " Left  hand q: " + str(list(map(lambda r:int((r*180/math.pi)*1000)/1000, ql))) + "       ")

    time.sleep(0.01)


if __name__ == "__main__":

 #   uni.connect_mqtt()
    ChannelFactoryInitialize(0, "enp2s0")
    pubR = ChannelPublisher(DEX3_RIGHT_CMD, HandCmd_)
    pubR.Init()
    
    pubL = ChannelPublisher(DEX3_LEFT_CMD, HandCmd_)
    pubL.Init()
  
    
    curses.wrapper(main)



      

