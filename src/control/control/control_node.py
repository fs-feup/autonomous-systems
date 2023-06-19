from rclpy.node import Node

import numpy as np 
import math
import rclpy

from custom_interfaces.msg import PointArray
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
from .abstraction_layer import create_abstraction_layer
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

        # Declare parameters.
        self.declare_parameter('mode', 'sim')
        
        # Steering angle velocity.
        self.steering_angle = 0.
        
        # Acceleration.
        self.acceleration = 0. # MAX_ACC / 2

        # Old error.
        self.old_error = 0

        # Path.
        self.path = None

        # Reference speeds
        self.speeds = None

        # old closest index
        self.old_closest_index = 0

        # Task completion
        self.done = False
        
        self.path_subscription = self.create_subscription(
            PointArray,
            'path_mock',
            self.path_callback,
            10
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.odometry_callback_mpc,
            10
        )
        
        # prevent unused variable warning
        self.path_subscription
        self.odom_subscription
    
        self.abstraction_layer = create_abstraction_layer(self)


    def odometry_callback_pid(self, msg):
        """!
        @brief Odometry callback.
        @param self The object pointer.
        @param msg Odometry message.
        """

        # self.get_logger().info("Received odom!")
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

        action = np.array([self.acceleration, self.steering_angle])
        state = np.array([position.x, position.y, lin_speed, yaw])
        new_action, closest_index2 = run_mpc(
            action, 
            state, 
            self.path, 
            self.old_closest_index
        )

        if new_action is None:
            return

        self.acceleration = new_action[0]
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