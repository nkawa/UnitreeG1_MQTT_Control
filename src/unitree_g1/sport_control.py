import time
import sys

from paho.mqtt import client as mqtt


from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class UnitreeG1_SportModeController:
  def __init__(self,  mqtt_client: mqtt.Client):
    self.sport_client = LocoClient()
    self.sport_client.SetTimeout(10.0)
    self.sport_client.Init()
    self.last_move = time.perf_counter()
    self.last_turn = time.perf_counter()
    
  
  def move(self, x, y):
    if time.perf_counter() - self.last_move < 0.8:
      return
    ret = self.sport_client.Move(-y,-x,0)
    print("Move ",y,x, ret)
    self.last_move = time.perf_counter()
#    if (x > y):
#      ret = self.sport_client.Move(0.3,0,0)
#    else:
#      ret = self.sport_client.Move(0, 0.3, 0)
    
  def turn(self, z):
    if time.perf_counter() - self.last_turn < 0.8:
      return
    ret = self.sport_client.Move(0,0,-z)
    print("Turn ",z, "ret", ret)
    self.last_turn = time.perf_counter()    
    

