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
  
  def move(self, x, y):
    self.sport_client.Move(x,y,0)
    
  def turn(self, z):
    self.sport_client.Move(0,0,z)
    
    

