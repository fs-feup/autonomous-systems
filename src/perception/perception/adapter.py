from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
import pyzed.sl as sl

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
            "/fsds/cameracam1/image_color",
            self.node.image_callback,
            qos_profile=qos_profile_sensor_data
        )

    def ads_dv_init(self):
        zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 60

        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        image = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        while True:
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT)
                zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
                self.node.image_callback(image)
            else:
                print("Error grabbing image!\n")
                break

        zed.close()
