import time

from unitree_sdk2py.core.channel import ChannelSubscriber,ChannelPublisher, ChannelFactoryInitialize

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC


class G1JointIndex:
    # Left leg
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5

    # Right leg
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11

    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked

    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21     # NOTE: INVALID for g1 23dof

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28    # NOTE: INVALID for g1 23dof

    kNotUsedJoint = 29 # NOTE: Weight


low_state = None
low_stateFirst = True
    
def LowStateHandler(msg: LowState_):
    global low_state,low_stateFirst
    low_state = msg
    if low_stateFirst:
        print("Get state!",msg)
        low_stateFirst = False

if __name__ == "__main__":

    ChannelFactoryInitialize(0, "enp2s0")

    pub  = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()

    lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    lowstate_subscriber.Init(LowStateHandler, 10)

    
    low_cmd = unitree_hg_msg_dds__LowCmd_()
    crc = CRC()

    arm_joints = [
          G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
          G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
          G1JointIndex.LeftWristRoll,      G1JointIndex.LeftWristPitch,
          G1JointIndex.LeftWristYaw,
          G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
          G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
          G1JointIndex.RightWristRoll,     G1JointIndex.RightWristPitch,
          G1JointIndex.RightWristYaw,
          G1JointIndex.WaistYaw,
          G1JointIndex.WaistRoll,
          G1JointIndex.WaistPitch
    ]

    time.sleep(3)

    low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1 # 1:Enable arm_sdk, 0:D
    

    for i, joint in enumerate(arm_joints):
        if joint == G1JointIndex.RightWristYaw:
            low_cmd.motor_cmd[joint].q = 0.05
        else:
            low_cmd.motor_cmd[joint].q = low_state.motor_state[joint].q

        mcmd = low_cmd.motor_cmd[joint]
        mcmd.tau = 0.
        mcmd.kp = 60.
        mcmd.dq = 0.
        mcmd.kd = 1.5
        mcmd.tau_ff = 0.
    
    
    low_cmd.crc = crc.Crc(low_cmd)
    
    pub.Write(low_cmd)

    time.sleep(3)

    low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1 # 1:Enable arm_sdk, 0:D
    low_cmd.motor_cmd[G1JointIndex.RightWristYaw].q = -0.02  # 1:Enable arm_sdk, 0:D
    
    low_cmd.crc = crc.Crc(low_cmd)
    pub.Write(low_cmd)

    time.sleep(3)

