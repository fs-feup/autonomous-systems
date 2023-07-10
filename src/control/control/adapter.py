from ackermann_msgs.msg import AckermannDriveStamped
from custom_interfaces.msg import VcuCommand, Vcu, Pose
from nav_msgs.msg import Odometry
from eufs_msgs.msg import CanState
from eufs_msgs.srv import SetCanState

import math
from .mpc_utils import wheels_vel_2_vehicle_vel
from tf_transformations import euler_from_quaternion
import rclpy
from .config import Params
import numpy as np

P = Params()

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

    def publish_cmd(self, steering_angle, speed, torque_req=0, break_req=0):
        if self.mode == "eufs":
            msg = AckermannDriveStamped()

            msg.drive.speed = speed
            msg.drive.steering_angle = steering_angle
            
        elif self.mode == "fsds":
            msg = AckermannDriveStamped()

            msg.drive.speed = speed
            msg.drive.steering_angle = steering_angle

        elif self.mode == "ads_dv":
            msg = VcuCommand()

            msg.axle_speed_request = 2 * speed * 60 / P.tire_diam
            msg.steering_angle_request = np.degrees(steering_angle)
            msg.axle_torque_request = torque_req
            msg.brake_press_request = break_req
              
        self.cmd_publisher.publish(msg)

    def eufs_init(self):
        self.cmd_publisher = self.node.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.mission_state_client = self.node.create_client(SetCanState, '/ros_can/set_mission')
        self.ebs = self.node.create_client(SetCanState, '/ros_can/ebs')

        # Example call to be removed later
        # self.eufs_set_mission_state(CanState.AMI_SKIDPAD, CanState.AS_READY)

        self.node.create_subscription(
            CanState,
            "/ros_can/state",
            self.eufs_mission_state_callback,
            10
        )
        self.node.create_subscription(
            Pose,
            "/vehicle_localization",
            self.localisation_callback,
            10
        )
        
    def fsds_init(self):
        self.cmd_publisher = self.node.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.mission_state_client = self.node.create_client(SetCanState, '/ros_can/set_mission')
        self.node.create_subscription(
            Pose,
            "/vehicle_localization",
            self.localisation_callback,
            10
        )

    def ads_dv_init(self):
        self.cmd_publisher = self.node.create_publisher(VcuCommand, "/cmd", 10)
        self.mission_state_client = self.node.create_client(SetCanState, '/ros_can/set_mission')
        self.node.create_subscription(
            Vcu,
            "/vcu",
            self.vcu_callback,
            10
        )
        self.node.create_subscription(
            Pose,
            "/vehicle_localization",
            self.localisation_callback,
            10
        )

    def localisation_callback(self, msg):
        position = msg.position
        yaw = msg.theta

        self.node.get_logger().info("[localisation] ({}, {}) {}".format(msg.position.x, msg.position.y, msg.theta))

        self.node.mpc_callback(position, yaw)
        # self.node.pid_callback(position, yaw)

    def eufs_odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        decomp_speed = msg.twist.twist.linear

        # get actual action variables
        self.node.velocity_actual = math.sqrt(decomp_speed.x**2 + decomp_speed.y**2)
        self.node.steering_angle_actual = self.node.steering_angle_command

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
            self.node.get_logger().info('Service call failed %r' % (future.exception(),))
    
    def vcu_callback(self, msg):
        self.node.steering_angle_actual = msg.steering_angle

        self.node.velocity_actual = wheels_vel_2_vehicle_vel(
            msg.fl_wheel_speed,
            msg.fr_wheel_speed,
            msg.rl_wheel_speed,
            msg.rr_wheel_speed,
            msg.steering_angle
        )
