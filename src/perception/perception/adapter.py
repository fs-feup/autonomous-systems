from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data

class PerceptionAdapter():
    def __init__(self, mode, node, withDepth = False):
        self.mode = mode
        self.node = node
        self.depth = withDepth

        if mode == "eufs":
            self.eufs_init(),
        elif mode == "fsds":
            self.fsds_init(),
        elif mode == "ads_dv":
            self.ads_dv_init()

        if self.depth == True:
            self.stereo_depth_init()

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
            "/fsds/cameracam1/image_color",
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

    def stereo_depth_init(self):
        self.node.create_subscription(
            Image,
            "/zed/left/image_rect_color",
            self.node.left_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.node.create_subscription(
            Image,
            "/zed/right/image_rect_color",
            self.node.right_callback,
            qos_profile=qos_profile_sensor_data
        )
