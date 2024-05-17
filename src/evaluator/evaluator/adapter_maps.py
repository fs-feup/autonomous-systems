from evaluator.adapter import Adapter
from evaluator.eufs_adapter import EufsAdapter
from evaluator.pacsim_adapter import PacsimAdapter
from evaluator.fsds_adapter import FSDSAdapter

ADAPTER_POINT_CLOUD_TOPIC_DICTINARY: dict[str, str] = {
    "fsds": "/lidar/Lidar1",
    "pacsim": "/no/topic",
    "eufs": "/velodyne_points",
}
ADAPTER_CONSTRUCTOR_DICTINARY: dict[str, Adapter] = {
    "fsds": FSDSAdapter,
    "pacsim": PacsimAdapter,
    "eufs": EufsAdapter,
}
