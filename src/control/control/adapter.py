from ackermann_msgs.msg import AckermannDriveStamped
from custom_interfaces.msg import VcuCommand, Vcu, Pose
from eufs_msgs.msg import CanState, WheelSpeedsStamped
from eufs_msgs.srv import SetCanState
from fs_msgs.msg import ControlCommand, GoSignal
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# used for testing purposes only
from nav_msgs.msg import Odometry

import math
from .mpc_utils import wheels_vel_2_vehicle_vel
from tf_transformations import euler_from_quaternion
import rclpy
from .config import Params
import numpy as np

P = Params()
TEST = False

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

    def publish_cmd(self, steering_angle=0., speed=0., torque_req=0, break_req=0, accel=0.):
        if self.mode == "eufs":
            msg = AckermannDriveStamped()

            # msg.drive.speed = float(speed)
            msg.drive.acceleration = float(accel)
            msg.drive.steering_angle = float(steering_angle)
            
        elif self.mode == "fsds":
            msg = ControlCommand()

            msg.throttle = torque_req / P.MAX_TORQUE # [0, 1]
            msg.drive.steering = steering_angle / P.MAX_STEER # [-1, 1]
            msg.brake = break_req / P.MAX_BREAK # [0, 1]

        elif self.mode == "ads_dv":
            msg = VcuCommand()

            msg.axle_speed_request = 2 * speed * 60 / P.tire_diam
            msg.steering_angle_request = np.degrees(steering_angle)
            msg.axle_torque_request = torque_req
            msg.brake_press_request = break_req

        self.cmd_publisher.publish(msg)

    def set_mission_state(self, mission, state):
        if self.mode == "eufs":
            self.eufs_set_mission_state(mission, state)
        elif self.mode == "fsds":
            # TODO
            return
        elif self.mode == "ads_dv":
            # TODO
            return

    def ebs(self):
        if self.mode == "eufs":
            self.eufs_ebs()
        elif self.mode == "fsds":
            # TODO
            return
        elif self.mode == "ads_dv":
            # TODO
            return

    def eufs_init(self):
        self.cmd_publisher =\
            self.node.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.mission_state_client =\
            self.node.create_client(SetCanState, "/ros_can/set_mission")
        self.ebs_client = self.node.create_client(Trigger, "/ros_can/ebs")

        # used only when interacting with the car
        self.driving_publisher =\
            self.node.create_publisher(Bool, "/state_machine/driving_flag", 10)
        self.driving_mission_publisher =\
            self.node.create_publisher(Bool, "/ros_can/mission_flag", 10)
        self.mission_completed_publisher =\
            self.node.create_publisher(Bool, "/ros_can/mission_completed", 10)

        self.node.create_subscription(
            CanState,
            "/ros_can/state",
            self.eufs_mission_state_callback,
            10
        )
        self.node.create_subscription(
            WheelSpeedsStamped,
            "/ros_can/wheel_speeds",
            self.eufs_odometry_callback,
            10
        )

        if True:
            self.node.create_subscription(
                Odometry,
                "/ground_truth/odom",
                self.eufs_sim_odometry_callback,
                10
            )
            self.node.create_subscription(
                Pose,
                "/vehicle_localization",
                self.localization_callback,
                10
            )

    def localization_callback(self, msg):
        position = msg.position
        yaw = msg.theta 
        
        # convert from [0, 2pi] to [-pi, pi]
        if yaw > math.pi:
            yaw -= 2 * math.pi

        # self.node.position = [position.x, position.y]

        self.node.get_logger().info(
            "[localization] ({}, {})\t{} rad".format(
                msg.position.x, msg.position.y, msg.theta
            )
        )

        if self.node.mission != CanState.AMI_AUTONOMOUS_DEMO:
            # self.node.mpc_callback(position, yaw)
            # self.node.pid_callback(position, yaw)
            pass

    def eufs_odometry_callback(self, msg, sim=False):
        self.node.wheel_speed = (msg.speeds.lb_speed +
            msg.speeds.rb_speed) / 2

        self.node.velocity_actual = wheels_vel_2_vehicle_vel(
            msg.speeds.lf_speed,
            msg.speeds.rf_speed,
            msg.speeds.lb_speed,
            msg.speeds.rb_speed,
            msg.speeds.steering
        )

        self.node.steering_angle_actual = msg.speeds.steering

    def eufs_mission_state_callback(self, msg):
        if self.node.state != CanState.AS_DRIVING and msg.as_state == CanState.AS_DRIVING:
            self.eufs_ready_to_drive_callback()

        self.node.mission = msg.ami_state
        self.node.state = msg.as_state

    def eufs_set_mission_state(self, mission, state):
        req = SetCanState.Request(ami_state=mission, as_state=state)

        future = self.mission_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info("Result: %d" % future.result().success)
        else:
            self.node.get_logger().info(
                "Service call failed %r" % (future.exception(),)
            )

    def eufs_mission_finished(self):
        self.node.get_logger().info("Mission finished!")
        msg = Bool()
        msg.data = True
        self.mission_completed_publisher.publish(msg)

    def eufs_ebs(self):
        self.node.get_logger().info("EBS triggered!")
        req = Trigger.Request()
        future = self.ebs_client.call_async(req)
        return future

    def eufs_ready_to_drive_callback(self):
        self.node.get_logger().info("Ready to drive callback!")
        diving_msg = Bool()
        diving_msg.data = True
        self.driving_publisher.publish(diving_msg)
        self.driving_mission_publisher.publish(diving_msg)
    
    # FSDS

    def fsds_init(self):
        self.cmd_publisher =\
            self.node.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.node.create_subscription(
            GoSignal,
            "/signal/go",
            self.fsds_state_callback,
            10
        )
        self.node.create_subscription(
            Pose,
            "/vehicle_localization",
            self.localization_callback,
            10
        )

    def fsds_state_callback(self, msg):
        return

    # ADS-DV

    def ads_dv_init(self):
        self.cmd_publisher = self.node.create_publisher(VcuCommand, "/cmd", 10)
        self.mission_state_client =\
            self.node.create_client(SetCanState, "/ros_can/set_mission")
        self.node.create_subscription(
            Vcu,
            "/vcu",
            self.vcu_callback,
            10
        )
        self.node.create_subscription(
            Pose,
            "/vehicle_localization",
            self.localization_callback,
            10
        )

    def vcu_callback(self, msg):
        self.node.steering_angle_actual = msg.steering_angle

        self.node.velocity_actual = wheels_vel_2_vehicle_vel(
            msg.fl_wheel_speed,
            msg.fr_wheel_speed,
            msg.rl_wheel_speed,
            msg.rr_wheel_speed,
            msg.steering_angle
        )

    # test reasons only
    def eufs_sim_odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        self.node.get_logger().info(
            "[eufs odom] ({}, {})\t{} rad".format(
                position.x, position.y, orientation
            )
        )

        decomp_speed = msg.twist.twist.linear

        self.node.position = position
        self.node.velocity_actual = math.sqrt(decomp_speed.x**2 + decomp_speed.y**2)
        self.node.steering_angle_actual = self.node.steering_angle_command

        # Converts quartenions base to euler's base, and updates the class' attributes
        yaw = euler_from_quaternion(orientation_list)[2]

        # self.node.mpc_callback(position, yaw)
        self.node.pid_callback(position, yaw)