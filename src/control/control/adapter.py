from ackermann_msgs.msg import AckermannDriveStamped
from custom_interfaces.msg import VcuCommand, Pose

from tf.transformations import euler_from_quaternion

class ControlAdapter():
    def __init__(self, mode, node):
        self.mode = mode
        self.node = node

        if mode == "eufs":
            self.eufs_init(),
        elif mode == "fsds":
            self.fsds_init(),
        elif mode == "ads_dv":
            self.ads_dv_init()

    def publish(self, steering_angle, speed):
        if self.mode == "eufs":
            msg = AckermannDriveStamped(
                steering_angle=steering_angle,
                speed=speed
            )
        elif self.mode == "fsds":
            msg = AckermannDriveStamped(
                steering_angle=steering_angle,
                speed=speed
            )
        elif self.mode == "ads_dv":
            msg = VcuCommand(
                steering_angle_request=steering_angle,
                axle_speed_request=speed
            )
        self.publisher.publish(msg)

    def eufs_init(self):
        self.publisher = self.node.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.node.create_subscription(
            Pose,
            "/odometry_integration/car_state",
            self.node.eufs_odometry_callback,
            10
        )
        
    def fsds_init(self):
        self.publisher = self.node.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.node.create_subscription(
            Pose,
            "/odometry_integration/car_state",
            self.node.eufs_odometry_callback,
            10
        )

    def ads_dv_init(self):
        self.publisher = self.node.create_publisher(VcuCommand, "/cmd", 10)
        self.node.create_subscription(
            Pose,
            "vehicle_localization",
            self.node.localisation_callback,
            10
        )

    def localisation_callback(self, msg):
        position = msg.position
        yaw = msg.orientation
        self.node.mpc_callback(position, yaw)

    def eufs_odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Converts quartenions base to euler's base, and updates the class' attributes
        yaw = euler_from_quaternion(orientation_list)[2]
        self.node.mpc_callback(position, yaw)
