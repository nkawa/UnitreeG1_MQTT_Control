import math
import json
import numpy as np
import time

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from .joint_index import G1JointIndex

from paho.mqtt import client as mqtt

ROBOT_UUID = "UnitreeG1-remote-vr"
MQTT_ROBOT_STATE_TOPIC = "robot/"+ROBOT_UUID

class UnitreeG1_JointMonitor:
  monitor_instance = None
  def __init__(self, mqtt_client: mqtt.Client):
      if UnitreeG1_JointMonitor.monitor_instance is not None:
        raise Exception("Only one instance of UnitreeG1_JointMonitor is allowed.")
        
      UnitreeG1_JointMonitor.monitor_instance = self
      self.client = mqtt_client    
#      ChannelFactoryInitialize(0, "enp2s0")
      ChannelFactoryInitialize(0, "eth0")  #  for unitree-g1 real device
      self.sub = ChannelSubscriber("rt/lowstate", LowState_)
      self.sub.Init(LowStateHandler, 10)
      self.left = None
      self.right = None
      self.low_state = None
    
    
  def publish_state(self, left,right):
      info = {
           "left": left,
           "right": right
      }
      self.left = np.array(left)
      self.right = np.array(right) #degree!
      self.client.publish(MQTT_ROBOT_STATE_TOPIC , json.dumps(info))
  
# LowState ハンドラ　⇒　モータ状態を MQTT で Publish
last_run = time.perf_counter()
interval = 0.02  # 20ms
call_count = 0

def LowStateHandler(msg: LowState_):
    global last_run, call_count
    uni = UnitreeG1_JointMonitor.monitor_instance
#    print("Version: ", msg.version, "Machine, mode ", msg.mode_machine, msg.mode_pr)
    #        print("Motor len:", len(msg.motor_state))
    ms = msg.motor_state
    uni.low_state = msg
    call_count += 1
    now = time.perf_counter()
    
    if now - last_run < interval:
        return
    
    leftrad = [ms[15].q,ms[16].q, ms[17].q, ms[18].q, ms[19].q ,ms[20].q, ms[21].q]
#    leftdeg = list(map(lambda r:int((r*180/math.pi)*1000)/1000, leftrad))
#    leftrad = list(map(lambda r:int(r*100000)/100000, leftrad))
    rightrad = [ms[22].q,ms[23].q, ms[24].q, ms[25].q, ms[26].q ,ms[27].q, ms[28].q]     
    rightdsp = list(map(lambda r:int(r*100000)/100000, rightrad))    
#    rightdeg = list(map(lambda r:int((r*180/math.pi)*1000)/1000, rightrad))        

    if call_count > 200:
#        print("R:",rightdsp, now-last_run, call_count)
        call_count = 0    
    uni.publish_state(leftrad, rightrad)
    last_run = now

    

