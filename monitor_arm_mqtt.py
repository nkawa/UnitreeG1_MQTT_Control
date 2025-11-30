import datetime
import logging
import json
import logging.handlers
import queue
import threading
import time
import math
import os

from paho.mqtt import client as mqtt
from dotenv import load_dotenv

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "control")


ROBOT_MODEL = os.getenv("ROBOT_MODEL","UnitreeG1-remote-vr")
ROBOT_UUID = os.getenv("ROBOT_UUID","UnitreeG1-remote-vr")

MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "mgr")
MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev")+"/"+ROBOT_UUID
MQTT_ROBOT_STATE_TOPIC = "robot/"+ROBOT_UUID
MQTT_MODE = os.getenv("MQTT_MODE", "metawork")



class UnitreeG1_MQTT:
   def __init__(self):
       self.mqtt_ctrl_topic = None
       self.last_registered = None

   def on_connect(self, client, userdata, connect_flags, reason_code, properties):
               # ロボットのメタ情報の中身はとりあえず
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

#"N       with self.mqtt_control_lock:
#           info["topic_type"] = "mgr/register"
#           info["topic"] = MQTT_MANAGE_TOPIC + "/register"
#           self.mqtt_control_dict.clear()
#           self.mqtt_control_dict.update(info)

       print("publish to: " + MQTT_MANAGE_TOPIC + "/register")
       self.last_registered = time.time()
       self.client.subscribe(MQTT_MANAGE_RCV_TOPIC)
       #       self.logger.info("subscribe to: " + MQTT_MANAGE_RCV_TOPIC)
       print("subscribe to: " + MQTT_MANAGE_RCV_TOPIC)

   def on_message(self, client, userdata, msg):
       print("From MQTT Get",userdata,msg)
       pass
           


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

   def publish_state(self, left,right):
       info = {
           "left": left,
           "right": right
       }
       self.client.publish(MQTT_ROBOT_STATE_TOPIC , json.dumps(info))

   def client_loop(self):
       self.client.loop_forever()

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

uni = UnitreeG1_MQTT()

def LowStateHandler(msg: LowState_):
    global uni
    print("Version: ", msg.version, "Machine, mode ", msg.mode_machine, msg.mode_pr)
    #        print("Wireless:", msg.wireless_remote)
    #        print("Motor:", msg.motor_state)

    #        print("Motor len:", len(msg.motor_state))
    ms = msg.motor_state
    #        msarray = [ms[13].q, ms[14].q, ms[15].q,ms[16].q, ms[17].q, ms[18].q, ms[19].q ]
    lmsarray = [ms[15].q,ms[16].q, ms[17].q, ms[18].q, ms[19].q ,ms[20].q, ms[21].q]
    leftdeg = list(map(lambda r:int((r*180/math.pi)*1000)/1000, lmsarray))
    rmsarray = [ms[22].q,ms[23].q, ms[24].q, ms[25].q, ms[26].q ,ms[27].q, ms[28].q]         
    rightdeg = list(map(lambda r:int((r*180/math.pi)*1000)/1000, rmsarray))        
#    print("LR:",leftdeg, rightdeg)
    print("msarray",rmsarray, lmsarray)

#    uni.publish_state(leftdeg, rightdeg)

if __name__ == "__main__":

    uni.connect_mqtt()
    ChannelFactoryInitialize(0, "enp2s0")
    sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
    sub.Init(LowStateHandler, 10)

    uni.client_loop()


