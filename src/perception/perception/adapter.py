from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

import pyzed.sl as sl
import cv2

class PerceptionAdapter():

    """!
    @brief Class that orchestrates the management of the runtime environment
    """

    def __init__(self, mode, node):

        """!
        @brief Constructor for selecting the environment
        @param self The object pointer
        @param mode The chosen environment option
        @param node The ROS Perception Node associated with the environment
        """

        self.mode = mode
        self.node = node

        if mode == "eufs":
            self.eufs_init(),
        elif mode == "fsds":
            self.fsds_init(),
        elif mode == "ads_dv":
            self.ads_dv_init()

    def eufs_init(self):

        """!
        @brief Initialization specific to the EUFS Simulator environment.
        @param self The object pointer
        """

        self.node.create_subscription(
            Image,
            "/zed/image_raw",
            self.node.image_callback,
            qos_profile=qos_profile_sensor_data
        )

    def fsds_init(self):

        """!
        @brief Initialization specific to the FSDS Simulator environment.
        @param self The object pointer
        """

        self.node.create_subscription(
            Image,
            "/fsds/cameracam1/image_color",
            self.node.image_callback,
            qos_profile=qos_profile_sensor_data
        )

    def ads_dv_init(self):

        """!
        @brief Initialization for the ADS-DV Vehicle environment with the ZED Camera
        @param self The object pointer
        """

        zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 60

        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        image = sl.Mat()
        point_cloud = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        while True:
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT)
                zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                ros_img = cv2.cvtColor(image.get_data(), cv2.COLOR_RGBA2RGB)
                self.node.image_callback(ros_img, sim=False, point_cloud=point_cloud)
            else:
                print("Error grabbing image!\n")

        zed.close()
