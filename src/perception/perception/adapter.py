from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

class PerceptionAdapter():
    def __init__(self, mode, node):
        self.mode = mode
        self.node = node

        if mode == "eufs":
            self.eufs_init(),
        elif mode == "fsds":
            self.fsds_init(),
        elif mode == "ads_dv":
            self.ads_dv_init()

    def eufs_init(self):
        self.node.create_subscription(
            Image,
            "/zed/image_raw",
            self.node.image_callback,
            qos_profile=qos_profile_sensor_data
        )
        
    def fsds_init(self):
        self.node.create_subscription(
            Image,
            "/zed/image_raw",
            self.node.image_callback,
            qos_profile=qos_profile_sensor_data
        )

    def ads_dv_init(self):
        self.node.create_subscription(
            Image,
            "/zed/image_raw",
            self.node.image_callback,
            qos_profile=qos_profile_sensor_data
        )
