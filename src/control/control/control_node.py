from rclpy.node import Node

import numpy as np 
import math
import rclpy

from custom_interfaces.msg import PointArray, VcuCommand
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from .pid_utils import (
    get_closest_point,
    get_position_error,
    get_orientation_error,
    get_cte,
    get_speed_command,
    steer
)
from .mpc import run_mpc
from .config import Params

P = Params()

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
        
        # self.create_subscription(
        #     Pose,
        #     'vehcile_localization',
        #     self.vehcile_localization_callback,
        #     10
        # )

        # Used for testing purposes on the simulator
        self.create_subscription(
            AckermannDriveStamped,
            '/ground_truth/odom',
            self.odometry_callback,
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

    def vehcile_localization_callback(self, msg):
        if self.path is None or self.done:
            return

        position = msg.position
        yaw = msg.orientation
        self.mpc_callback(position, yaw)

    def odometry_callback(self, msg):
        if self.path is None or self.done:
            return

        # get pose feedback
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Converts quartenions base to euler's base, and updates the class' attributes
        yaw = euler_from_quaternion(orientation_list)[2]
        self.mpc_callback(position, yaw)

    def pid_callback(self, position, yaw):
        """!
        @brief Odometry callback.
        @param self The object pointer.
        @param msg Odometry message.
        """

        # get postion reference
        closest_index = get_closest_point(
            [position.x, position.y],
            self.path,
            self.old_closest_index,
            search_window=7
        )

        # gets position error
        pos_error = get_position_error(
            [position.x, position.y, yaw],
            self.path[closest_index]
        )

        # gets orientation error
        yaw_error = get_orientation_error(closest_index, self.path, yaw)

        # gets cross track error
        ct_error  = get_cte(closest_index, self.path, [position.x, position.y, yaw])

        self.steering_angle, self.old_error = steer(
            pos_error, yaw_error, ct_error, self.old_error
        )

        self.velocity = get_speed_command(self.speeds, closest_index)

        self.old_closest_index = closest_index

    def mpc_callback(self, position, yaw):
        action = np.array([self.velocity, self.steering_angle])
        state = np.array([position.x, position.y, yaw])

        new_action, closest_index = run_mpc(
            action, 
            state, 
            self.path, 
            self.old_closest_index
        )

        if new_action is None:
            return

        self.velocity = new_action[0]
        self.steering_angle = new_action[1]

        self.old_closest_index = closest_index


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
            speed = P.VEL*min(1.0, dist_from_end / P.START_BREAKING_POS)
            speeds.append(speed)

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