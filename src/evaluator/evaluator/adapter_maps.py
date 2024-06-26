from evaluator.adapter import Adapter
from evaluator.eufs_adapter import EufsAdapter
from evaluator.pacsim_adapter import PacsimAdapter
from evaluator.fsds_adapter import FSDSAdapter
from evaluator.evaluator.vehicle_adapter import VehicleAdapter

ADAPTER_POINT_CLOUD_TOPIC_DICTINARY: dict[str, str] = {
    "fsds": "/lidar/Lidar1",
    "pacsim": "/no/topic",
    "eufs": "/velodyne_points",
    "vehicle_preprocessed": "/rslidar_points/pre_processed",
}
ADAPTER_CONSTRUCTOR_DICTINARY: dict[str, Adapter] = {
    "fsds": FSDSAdapter,
    "pacsim": PacsimAdapter,
    "eufs": EufsAdapter,
    "vehicle_preprocessed": VehicleAdapter
}
