from evaluator.adapter import Adapter
from evaluator.eufs_adapter import EufsAdapter
from evaluator.pacsim_adapter import PacsimAdapter
from evaluator.fsds_adapter import FSDSAdapter
from evaluator.onground_adapter import OnGroundAdapter
from evaluator.vehicle_adapter import VehicleAdapter
import rclpy
import rclpy.qos

ADAPTER_POINT_CLOUD_TOPIC_DICTIONARY: dict[str, str] = {
    "fsds": "/lidar/Lidar1",
    "pacsim": "/no/topic",
    "eufs": "/velodyne_points",
    "vehicle": "/rslidar_points",
    "onground": "/rslidar_points",
}
ADAPTER_POINT_CLOUD_TOPIC_QOS_DICTIONARY: dict[str, rclpy.qos.QoSProfile] = {
    "fsds": rclpy.qos.QoSProfile(depth=10),
    "pacsim": rclpy.qos.QoSProfile(depth=10),
    "eufs": rclpy.qos.QoSProfile(
        depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT
    ),
    "vehicle_preprocessed": rclpy.qos.QoSProfile(depth=10),
}
ADAPTER_CONSTRUCTOR_DICTINARY: dict[str, Adapter] = {
    "fsds": FSDSAdapter,
    "pacsim": PacsimAdapter,
    "eufs": EufsAdapter,
    "vehicle": VehicleAdapter,
    "onground": OnGroundAdapter,
}
