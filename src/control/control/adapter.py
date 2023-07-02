from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from custom_interfaces.msg import VcuCommand, Pose
from eufs_msgs.msg import CanState
from eufs_msgs.srv import SetCanState

from tf_transformations import euler_from_quaternion
import rclpy

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

    def publish_cmd(self, steering_angle, speed):
        if self.mode == "eufs":
            msg = AckermannDriveStamped()
            msg.drive.steering_angle = steering_angle
            msg.drive.speed = speed

        elif self.mode == "fsds":
            msg = AckermannDriveStamped()
            msg.drive.steering_angle = steering_angle
            msg.drive.speed = speed

        elif self.mode == "ads_dv":
            msg = VcuCommand(
                steering_angle_request=steering_angle,
                axle_speed_request=speed
            )
        self.cmd_publisher.publish(msg)

    def eufs_init(self):
        self.cmd_publisher = self.node.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.mission_state_client = self.node.create_client(SetCanState, '/ros_can/set_mission')
        self.ebs = self.node.create_client(SetCanState, '/ros_can/ebs')

        # Example call to be removed later
        # self.eufs_set_mission_state(CanState.AMI_SKIDPAD, CanState.AS_READY)

        self.node.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self.eufs_odometry_callback,
            10
        )
        self.node.create_subscription(
            CanState,
            "/ros_can/state",
            self.eufs_mission_state_callback,
            10
        )
        
    def fsds_init(self):
        self.cmd_publisher = self.node.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.mission_state_client = self.node.create_client(SetCanState, '/ros_can/set_mission')
        self.node.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self.eufs_odometry_callback,
            10
        )

    def ads_dv_init(self):
        self.cmd_publisher = self.node.create_publisher(VcuCommand, "/cmd", 10)
        self.mission_state_client = self.node.create_client(SetCanState, '/ros_can/set_mission')
        self.node.create_subscription(
            Pose,
            "vehicle_localization",
            self.localisation_callback,
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

    def eufs_mission_state_callback(self, msg):
        mission = msg.ami_state
        state = msg.as_state

        self.node.mission_state_callback(mission, state)

    def eufs_set_mission_state(self, mission, state):
        req = SetCanState.Request(as_state=state, ami_state=mission)

        future = self.mission_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info('Result: %d' % future.result().success)
        else:
            self.node.get_logger().info('Service call failed %r' % (result.exception(),))
