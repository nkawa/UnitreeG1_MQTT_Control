import time

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

#uni = UnitreeG1_MQTT()

DEX3_LEFT = "rt/dex3/left"
DEX3_LEFT_STATE = "rt/dex3/left/state"

DEX3_RIGHT = "rt/dex3/right"
DEX3_RIGHT_STATE = "rt/dex3/right/state"

MAX_LIMITS_LEFT =  [  1.05 ,  1.05  , 1.75 ,   0   ,  0    , 0     , 0   ] # // set max motor value
MIN_LIMITS_LEFT =  [ -1.05 , -0.724 ,   0  , -1.57 , -1.75 , -1.57  ,-1.75 ]
MAX_LIMITS_RIGHT = {  1.05 , 0.742  ,   0  ,  1.57 , 1.75  , 1.57  , 1.75};
MIN_LIMITS_RIGHT = { -1.05 , -1.05  , -1.75,    0  ,  0    ,   0   ,0    };

def HandStateHandler(msg: HandState_):
    global uni
    #        print("Wireless:", msg.wireless_remote)
    print("Motor:", msg.motor_state)
    print("Press:", msg.press_sensor_state)
    print("IMU  :", msg.imu_state)
    print("p_v,p_a, s_v, dev_d:", msg.power_v, msg.power_a, msg.system_v, msg.device_d)

    #        print("Motor len:", len(msg.motor_state))
    ms = msg.motor_state

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
    sub = ChannelSubscriber(DEX3_RIGHT_STATE, HandState_)
    sub.Init(HandStateHandler, 10)

    while True:
      time.sleep(10)
      

