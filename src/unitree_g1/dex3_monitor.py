import math
import json
import numpy as np


from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_



class UnitreeG1_Dex3Monitor:
  monitor_instance = None
  def __init__(self, mqtt_client: mqtt.Client):
      if UnitreeG1_Dex3Monitor.monitor_instance is not None:
        raise Exception("Only one instance of UnitreeG1_Dex3Monitor is allowed.")
        
      UnitreeG1_Dex3Monitor.monitor_instance = self
      self.client = mqtt_client    
      ChannelFactoryInitialize(0, "enp2s0")
      self.sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
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
def LowStateHandler(msg: LowState_):
    uni = UnitreeG1_JointMonitor.monitor_instance
#    print("Version: ", msg.version, "Machine, mode ", msg.mode_machine, msg.mode_pr)
    #        print("Motor len:", len(msg.motor_state))
    ms = msg.motor_state
    uni.low_state = msg
    leftrad = [ms[15].q,ms[16].q, ms[17].q, ms[18].q, ms[19].q ,ms[20].q, ms[21].q]
#    leftdeg = list(map(lambda r:int((r*180/math.pi)*1000)/1000, leftrad))
    leftrad = list(map(lambda r:int(r*100000)/100000, leftrad))
    rightrad = [ms[22].q,ms[23].q, ms[24].q, ms[25].q, ms[26].q ,ms[27].q, ms[28].q]     
    rightrad = list(map(lambda r:int(r*100000)/100000, rightrad))    
#    rightdeg = list(map(lambda r:int((r*180/math.pi)*1000)/1000, rightrad))        
    print("R:",rightrad)

    uni.publish_state(leftrad, rightrad)
    

