from evaluator.adapter import Adapter
from evaluator.onground_adapter import OnGroundAdapter
from evaluator.vehicle_adapter import VehicleAdapter
import rclpy
import rclpy.qos

ADAPTER_POINT_CLOUD_TOPIC_DICTIONARY: dict[str, str] = {
    "vehicle": "/rslidar_points",
    "onground": "/rslidar_points",
}
ADAPTER_POINT_CLOUD_TOPIC_QOS_DICTIONARY: dict[str, rclpy.qos.QoSProfile] = {
    "vehicle_preprocessed": rclpy.qos.QoSProfile(depth=10),
}
ADAPTER_CONSTRUCTOR_DICTINARY: dict[str, Adapter] = {
    "vehicle": VehicleAdapter,
    "onground": OnGroundAdapter,
}
