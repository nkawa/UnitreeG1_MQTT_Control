SHM_NAME = "unitreeg1_shm"
SHM_SIZE = (7*2+7*2)*2+4  # joint pos/vel x7 + imu acc/gyro x4

class SHM_INDEX:
    LEFT_JOINT_INFO = 0  # 0-6
    RIGHT_JOINT_INFO = 7 # 7-13
    LEFT_JOINT_TGT = 14  # 28-34
    RIGHT_JOINT_INFO = 21# 35-41
    LEFT_HAND_INFO = 28  # 14-20
    RIGHT_HAND_INFO = 35 # 21-27
    LEFT_HAND_TGT = 42   # 42-48
    RIGHT_HAND_TGT = 49 # 49-55
    OTHER_INFO = 56    # 56-59 
       