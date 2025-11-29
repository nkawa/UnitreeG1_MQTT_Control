#this time we do not use GUI

import json
import time
import datetime
from paho.mqtt import client as mqtt
from unitree_g1.joint_controller import UnitreeG1_JointController
from unitree_g1.joint_monitor import UnitreeG1_JointMonitor
from unitree_g1.dex3_mon_cont import UnitreeG1_Dex3MonitorController
from unitree_g1.config import SHM_NAME, SHM_SIZE, SHM_INDEX
from mqtt_config import *
import numpy as np

# マルチプロセス対応
# プロセス間通信は Shared Memory で
#  0: MQTT Process
#  1: Joint Monitor Process　 -> 1kHz で情報を受け取るけど publish は 20Hz程度
#  2: Joint Controller Process -> 500Hz くらいで制御情報を送りたい
import multiprocessing as mp
from multiprocessing import Process

class UnitreeG1_MQTT:
  def __init__(self):
       self.mqtt_ctrl_topic = None  # 現在制御 を受けているゴーグル・ブラウザのトピック
       self.last_registered = None
       self.client = None

  def on_connect(self, client, userdata, connect_flags, reason_code, properties):
               # ロボットのメタ情報の中身はとりあえず
      print("MQTT Connected. then subscribe:" + MQTT_MANAGE_RCV_TOPIC)
      self.client.subscribe(MQTT_MANAGE_RCV_TOPIC)     # 登録応答を受け取るためにサブスクライブ

      date = datetime.datetime.now().strftime('%c')
      info = {
           "date": date,
           "device": {
                "agent": "none",
                "cookie": "none",
           },
           "devType": "robot",
           "type": ROBOT_MODEL,
           "version": "0.1",
           "devId": ROBOT_UUID,
      }

      self.client.publish(MQTT_MANAGE_TOPIC + "/register", json.dumps(info))

      print("publish to: " + MQTT_MANAGE_TOPIC + "/register")
      self.last_registered = time.time()
      self.client.subscribe(MQTT_MANAGE_RCV_TOPIC)
       
  def on_message(self, client, userdata, msg):
#      print("From MQTT Get",userdata,msg)
      if(msg.topic == MQTT_MANAGE_RCV_TOPIC): # 制御元のVRゴーグル・ブラウザからのメッセージ
          print("Get Manage message:", msg.topic, msg.payload)
          js = json.loads(msg.payload)
          goggles_id = js["devId"]
          mqtt_ctrl_topic = MQTT_CTRL_TOPIC + "/" + goggles_id
          if mqtt_ctrl_topic != self.mqtt_ctrl_topic:
            if self.mqtt_ctrl_topic is not None:
              self.client.unsubscribe(self.mqtt_ctrl_topic)    
          self.mqtt_ctrl_topic = mqtt_ctrl_topic
          self.client.subscribe(self.mqtt_ctrl_topic)
          print("MQTT Subscribe to: " + self.mqtt_ctrl_topic)

      elif msg.topic == self.mqtt_ctrl_topic : # 制御コマンド受信
#          print("Control command received:", msg.topic)
          js = json.loads(msg.payload)
          if 'arm' in js :
              arm = js['arm']
              if arm  == 'right':
                  right = js['joints']
                  self.joint_controller.send_right_arm_command(right)
                  if js['button'][0]: # close
                        self.joint_controller.send_right_hand_command(1)
                  elif js['button'][1]: # open
                        self.joint_controller.send_right_hand_command(-1)
                
              elif arm == 'left':
                  left = js['joints']
                  self.joint_controller.save_left_arm_command(left)
                  if js['button'][0]: # close
                        self.joint_controller.send_left_hand_command(1)
                  elif js['button'][1]: # open
                        self.joint_controller.send_left_hand_command(-1)   
          else:
              print("Invalid joint command message:", js)
      else:
          print("Unknown topic message:", msg.topic, msg.payload)
  
  def on_disconnect(
        self,
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties,
       ):
       if reason_code != 0:
          print("MQTT Unexpected disconnection.")
          
  
  def connect_mqtt(self):
       self.client = mqtt.Client(
           callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
       # MQTTの接続設定
       self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
       self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
       self.client.on_message = self.on_message         # メッセージ到着時のコールバック
       self.client.connect(MQTT_SERVER, 1883, 60)
#       self.client.loop_start()   # 通信処理開始

  def client_loop(self):
       self.client.loop_forever()
       
  def setJointControl(self, joint_controller: UnitreeG1_JointController):
      self.joint_controller = joint_controller      


##
# 　複数プロセスをまとめる！

class ProcessManager:
    def __init__(self):
        self.monP = None # JointMonitor プロセス
        self.conrolP = None # JointController プロセス
        self.UniMQ = None # MQTT インスタンス
        self.dex3_mon_con = None
        
#        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
 #       self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        

    def start_mqtt(self): # MQTT メイン
        self.UniMQ = UnitreeG1_MQTT()
        self.UniMQ.connect_mqtt()
    
    def start_monitor(self, logging_dir: str | None = None, disable_mqtt: bool = False):
        self.mon = UnitreeG1_JointMonitor(self.UniMQ.client)
#        self.monP = Process(
#            target=self.mon.run_proc,
#            args=(self.monitor_dict,
#                  self.monitor_lock,
##                  self.log_queue,
 #                 self.monitor_pipe,
 #                 logging_dir,
 #                 disable_mqtt),
 #           name="UnitreeG1_monitor")
 #       self.monP.start()
  #      self.state_monitor = True
  
    def start_joint_controller(self):
        self.joint_controller = UnitreeG1_JointController()
        self.set_joint_control(self.joint_controller)
  
    def set_joint_control(self, joint_controller: UnitreeG1_JointController):
        self.UniMQ.setJointControl(joint_controller)
    
    def start_dex3_monitor_controller(self):
        self.dex3_mon_con = UnitreeG1_Dex3MonitorController(self.UniMQ.client)
        
    def start_mqtt_loop(self):
        self.UniMQ.client_loop()

if __name__ == '__main__':  
  unipro = ProcessManager()
  unipro.start_mqtt()  # registration
  unipro.start_monitor()
  unipro.start_joint_controller()
  
  unipro.start_dex3_monitor_controller()
    
  unipro.start_mqtt_loop()
  

