import time
import math

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import PressSensorState_
#from unitree_sdk2py.idl.unitree_hg.msg.dds_ import PressureSensorState_

#uni = UnitreeG1_MQTT()

DEX3_LEFT = "rt/dex3/left"
DEX3_LEFT_STATE = "rt/lf/dex3/left/state"

DEX3_RIGHT = "rt/dex3/right"
DEX3_RIGHT_STATE = "rt/lf/dex3/right/state"
#DEX3_RIGHT_STATE = "rt/dex3/right/state"

MAX_LIMITS_LEFT =  [  1.05 ,  1.05  , 1.75 ,   0   ,  0    , 0     , 0   ] # // set max motor value
MIN_LIMITS_LEFT =  [ -1.05 , -0.724 ,   0  , -1.57 , -1.75 , -1.57  ,-1.75 ]
MAX_LIMITS_RIGHT = {  1.05 , 0.742  ,   0  ,  1.57 , 1.75  , 1.57  , 1.75};
MIN_LIMITS_RIGHT = { -1.05 , -1.05  , -1.75,    0  ,  0    ,   0   ,0    };

last_time = time.perf_counter()

def HandStateHandler(msg: HandState_):
   global uni
   global last_time
   
   curr_time = time.perf_counter()
   print("Diff Time:", int((curr_time - last_time)*1000000)/1000, "ms")  
   last_time = curr_time
#        print("Wireless:", msg.wireless_remote)
   print("Press:", len(msg.press_sensor_state), msg.press_sensor_state[0].pressure)
#   print("IMU  :", msg.imu_state)
#   print("p_v,p_a, s_v, dev_d:", msg.power_v, msg.power_a, msg.system_v, msg.device_v)

    #        print("Motor len:", len(msg.motor_state))
   ms = msg.motor_state
    
   msarray = [ ms[0].q, ms[1].q, ms[2].q, ms[3].q, ms[4].q ,ms[5].q, ms[6].q ]
#    print("Motor:", msg.motor_state)
   print("Right:", list(map(lambda r:int((r*180/math.pi)*1000)/1000, msarray)))   
   print("RightDeg:",  msarray)  
   print("Mode", ms[0].mode, ms[1].mode, ms[2].mode, ms[3].mode, ms[4].mode ,ms[5].mode, ms[6].mode )

    #        msarray = [ms[13].q, ms[14].q, ms[15].q,ms[16].q, ms[17].q, ms[18].q, ms[19].q ]
#    msarray = [ms[15].q,ms[16].q, ms[17].q, ms[18].q, ms[19].q ,ms[20].q, ms[21].q]
#    leftdeg = list(map(lambda r:int((r*180/math.pi)*1000)/1000, msarray))
#    msarray = [ms[22].q,ms[23].q, ms[24].q, ms[25].q, ms[26].q ,ms[27].q, ms[28].q]         
#    rightdeg = list(map(lambda r:int((r*180/math.pi)*1000)/1000, msarray))        
#    print("LR:",leftdeg, rightdeg)

#    uni.publish_state(leftdeg, rightdeg);

if __name__ == "__main__":

 #   uni.connect_mqtt()
    ChannelFactoryInitialize(0, "enp2s0")
#    subR = ChannelSubscriber(DEX3_RIGHT_STATE, HandState_)
    subL = ChannelSubscriber(DEX3_LEFT_STATE, HandState_)
    subL.Init(HandStateHandler, 10)

    while True:
      time.sleep(10)
      

