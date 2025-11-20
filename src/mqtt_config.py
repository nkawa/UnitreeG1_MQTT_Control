import os
from dotenv import load_dotenv

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "control")


ROBOT_MODEL = os.getenv("ROBOT_MODEL","UnitreeG1-remote-vr")
ROBOT_UUID = os.getenv("ROBOT_UUID","UnitreeG1-remote-vr")

MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "mgr")
MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev")+"/"+ROBOT_UUID

