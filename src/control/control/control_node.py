from rclpy.node import Node

import numpy as np 
import math
import rclpy

from custom_interfaces.msg import PointArray, VcuCommand, Imu
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from .pid_utils import (
    get_closest_point,
    get_position_error,
    get_orientation_error,
    get_cte,
    get_reference_speed,
    get_speed_error,
    steer,
    accelerate
)
from .mpc import run_mpc
from .config import Params

P = Params()

test_config = {
    "path_topic": "path_mock",
    "odometry_topic": "/ground_truth/odometry",
}

real_config = {
    
}

config = test_config

class ControlNode(Node):
    """!
    @brief Class that controls the car's steering and speed.
    """

    def __init__(self):
        """!
        @brief Constructor of the class.
        @param self The object pointer.
        """
        super().__init__('control_node')
        
        # Steering angle.
        self.steering_angle = 0.
        
        # Velocity.
        self.velocity = 0.

        # Old error.
        self.old_error = 0.

        # Path.
        self.path = None

        # Reference speeds
        self.speeds = None

        # old closest index
        self.old_closest_index = 0

        # Task completion
        self.done = False
        
        self.create_subscription(
            PointArray,
            'path_mock',
            self.path_callback,
            10
        )
        
        self.create_subscription(
            Imu,
            'vehcile_info',
            self.odometry_callback_mpc,
            10
        )

        timer_period = 0.2  # seconds
        node.timer = node.create_timer(timer_period, self.timer_callback)
        self.node.create_publisher(AckermannDriveStamped, "vehcile_command", 10)

    def timer_callback(self):
        """!
        @brief Sim publisher callback.
        @param self The object pointer.
        """
        node = self.node
        
        cmd_msg = VcuCommand()

        # TODO: Set the minimum and maximum steering angle
        cmd_msg.steering_angle_request = node.steering_angle if not node.done else 0.
        cmd_msg.axle_speed_request = node.velocity if not node.done else -1.

        node.publisher_.publish(cmd_msg)

        node.get_logger().info('Published Vehicle Command')

    def odometry_callback_pid(self, msg):
        """!
        @brief Odometry callback.
        @param self The object pointer.
        @param msg Odometry message.
        """

        if self.path is None or self.done:
            return

        # get pose feedback
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        # get speed feedback
        speed = msg.twist.twist.linear
        lin_speed = math.sqrt(speed.x**2 + speed.y**2)

        # Converts quartenions base to euler's base, and updates the class' attributes
        yaw = euler_from_quaternion(orientation_list)[2]

        # get postion reference
        closest_point, closest_index = get_closest_point(
            [position.x, position.y],
            self.path,
        )

        # get speed reference
        ref_speed = get_reference_speed(self.speeds, closest_index)

        # gets position error
        pos_error = get_position_error([position.x, position.y, yaw], closest_point)

        # gets orientation error
        yaw_error = get_orientation_error(closest_index, self.path, yaw)

        # gets cross track error
        ct_error = get_cte(closest_index, self.path, [position.x, position.y, yaw])

        # get speed error
        speed_error = get_speed_error(lin_speed, ref_speed)

        self.steering_angle, self.old_error = steer(
            pos_error, yaw_error, ct_error, self.old_error
        )
        self.acceleration = accelerate(speed_error)

        if lin_speed < 0.2 and closest_index == len(self.path) - 1 and not self.done:
            self.done = True

        # show info
        self.get_logger().info(f"\n\n\n\npos error: {pos_error}"+
            f"\nyaw error: {yaw_error}," + 
            f"\ncross track error: {ct_error}," + 
            f"\nsteerin angle velocity: {self.steering_angle}," + 
            f"\nspeed: {lin_speed}," + 
            f"\nclosest: {closest_point},"
            f"\nposition: {(position.x, position.y, yaw)}," +
            f"\ndone: {self.done}")

    def odometry_callback_mpc(self, msg):
        if self.path is None or self.done:
            return

        # get pose feedback
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        # get speed feedback
        speed = msg.twist.twist.linear
        lin_speed = math.sqrt(speed.x**2 + speed.y**2)

        # Converts quartenions base to euler's base, and updates the class' attributes
        yaw = euler_from_quaternion(orientation_list)[2]

        # get postion reference
        closest_point, closest_index = get_closest_point(
            [position.x, position.y],
            self.path,
        )

        if lin_speed < 0.2 and \
            closest_index == len(self.path) - 1 and \
            not self.done and False:
            self.done = True

        action = np.array([self.velocity, self.steering_angle])
        state = np.array([position.x, position.y, yaw])
        new_action, closest_index2 = run_mpc(
            action, 
            state, 
            self.path, 
            self.old_closest_index
        )

        print(f"index 1: {closest_index}, index 2: {closest_index2}")

        if new_action is None:
            return

        self.velocity = new_action[0]
        self.steering_angle = new_action[1]

        self.old_closest_index = closest_index2

        # show info
        """
        self.get_logger().info(f"\n\n\n"+
            f"\nsteerin angle velocity: {self.steering_angle}," + 
            f'\nacceleration: {self.acceleration}'
            f"\nspeed: {lin_speed}," + 
            f"\nclosest: {closest_point},"
            f"\nposition: {(position.x, position.y, yaw)}," +
            f"\ndone: {self.done}")
        """


    def path_callback(self, points_list):
        """!
        @brief Path callback.
        @param self The object pointer.
        @param path Path message.
        """
        self.get_logger().info("[received] {} points".format(len(points_list.points)))
        path = []
        speeds = []
        
        for i, point in enumerate(points_list.points):
            path.append([point.x, point.y])

            dist_from_end = len(points_list.points) - i

            # min -> start breaking or not
            speed = P.VEL*min(1, dist_from_end / P.START_BREAKING_POS)
            speeds.append([speed])

        self.path = np.array(path)
        self.speeds = np.array(speeds)


def main(args=None):
    rclpy.init(args=args)

    control_node = ControlNode()

    rclpy.spin(control_node)

    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()