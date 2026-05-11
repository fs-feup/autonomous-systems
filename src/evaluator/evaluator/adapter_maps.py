from evaluator.adapter import Adapter
from evaluator.pacsim_adapter import PacsimAdapter
from evaluator.onground_adapter import OnGroundAdapter
from evaluator.vehicle_adapter import VehicleAdapter
import rclpy
import rclpy.qos

ADAPTER_POINT_CLOUD_TOPIC_DICTIONARY: dict[str, str] = {
    "pacsim": "/no/topic",
    "vehicle": "/rslidar_points",
    "onground": "/rslidar_points",
}
ADAPTER_POINT_CLOUD_TOPIC_QOS_DICTIONARY: dict[str, rclpy.qos.QoSProfile] = {
    "pacsim": rclpy.qos.QoSProfile(depth=10),
    "vehicle_preprocessed": rclpy.qos.QoSProfile(depth=10),
}
ADAPTER_CONSTRUCTOR_DICTINARY: dict[str, Adapter] = {
    "pacsim": PacsimAdapter,
    "vehicle": VehicleAdapter,
    "onground": OnGroundAdapter,
}
