from rclpy.node import Node

import numpy as np 
import rclpy

from custom_interfaces.msg import PointArray
from eufs_msgs.msg import CanState

from .pid_utils import (
    get_closest_point,
    get_position_error,
    get_orientation_error,
    get_cte,
    get_torque_break_commands,
    get_acceleration_command,
    get_steering_command,
    get_reference_speed,
    get_speed_error
)
from .mpc import run_mpc
from .config import Params
from .adapter import ControlAdapter
from .inspection import ddt_inspection_a, ddt_inspection_b, autonomous_demo_timed

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
        
        self.mission = CanState.AMI_NOT_SELECTED

        self.state = CanState.AS_OFF

        self.position = None

        # Steering angle.
        self.steering_angle_command = 0.

        # Actual steering angle
        self.steering_angle_actual = 0.
        
        # Velocity command.
        self.acceleration_command = 0.

        # Actual car velocity
        self.velocity_actual = 0.

        # Old steer error.
        self.old_steer_error = 0.

        # Old velocity error
        self.old_velocity_error = 0.

        # Velocity error list
        self.velocity_error_list = []

        # Path.
        self.path = None

        # Reference speeds
        self.path_speeds = None

        # old closest index
        self.old_closest_index = 0

        # Task completion
        self.done = False

        self.create_subscription(
            PointArray,
            "/planning_local",
            self.path_callback,
            10
        )
        self.adapter = ControlAdapter(P.env, self)

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        """!
        @brief Sim publisher callback.
        @param self The object pointer.
        """

        if self.state != CanState.AS_DRIVING:
            return

        if self.mission == CanState.AMI_ACCELERATION or \
            self.mission == CanState.AMI_SKIDPAD or \
            self.mission == CanState.AMI_AUTOCROSS or \
            self.mission == CanState.AMI_TRACK_DRIVE:
            self.dynamic_callback()
        else:
            self.inspection_callback()

    def dynamic_callback(self):
        if self.done:
            self.adapter.publish_cmd(
                steering_angle = 0.,
                speed = 0.,
                torque_req = 0.,
                break_req = 100.,
                accel = 0.
            )
            return

        # after mpc(velocity version), convert velocity command to torque/break command
        torque_command, break_command = \
            get_torque_break_commands(self.acceleration_command)


        # clip to limits
        steering_angle_command = np.clip(self.steering_angle_command, 
                                         -P.MAX_STEER, P.MAX_STEER)
        self.acceleration_command = \
            np.clip(self.acceleration_command, 0, P.MAX_ACC)
        torque_command = np.clip(torque_command, 0, P.MAX_TORQUE)
        break_command = np.clip(break_command, 0, P.MAX_BREAK)

        self.adapter.publish_cmd(steering_angle_command,
            accel=self.acceleration_command,
            torque_req=torque_command,
            break_req=break_command
        )

    def inspection_callback(self):
        if self.mission == CanState.AMI_DDT_INSPECTION_A:
            ddt_inspection_a(self)
        elif self.mission == CanState.AMI_DDT_INSPECTION_B:
            ddt_inspection_b(self)
        elif self.mission == CanState.AMI_AUTONOMOUS_DEMO:
            # autonomous_demo(self)
            autonomous_demo_timed(self)
        else:
            self.get_logger().info("Inspection not selected")

    def pid_callback(self, position, yaw):
        """!
        @brief Odometry callback.
        @param self The object pointer.
        @param msg Odometry message.
        """
        if self.path is None or len(self.path) == 0 or self.done:
            return

        # get position reference
        closest_index = get_closest_point(
            [position.x, position.y],
            self.path,
            self.old_closest_index,
            search_window=7
        )

        # get velocity reference
        ref_speed = get_reference_speed(self.path_speeds, closest_index)

        # get position error
        pos_error = get_position_error(
            [position.x, position.y, yaw],
            self.path[closest_index]
        )

        # get orientation error
        yaw_error = get_orientation_error(closest_index, self.path, yaw)

        # get cross track error
        ct_error  = get_cte(closest_index, self.path, [position.x, position.y, yaw])

        # get speed error
        speed_error = get_speed_error(ref_speed, self.velocity_actual)

        # get steering command
        self.steering_angle_command, self.old_steer_error = get_steering_command(
            pos_error, yaw_error, ct_error, self.old_steer_error
        )

        # get acceleration command
        self.acceleration_command = get_acceleration_command(speed_error)

        self.old_closest_index = closest_index

        # set done
        to_end = [
            position.x - self.path[-1][0],
            position.y - self.path[-1][1]
        ]
        if (np.linalg.norm(to_end) <= P.done_trigger_dist) and \
            (closest_index == (len(self.path) - 1)):
            self.done = True
            self.adapter.eufs_mission_finished()


    def mpc_callback(self, position, yaw):
        if self.path is None or self.done:
            return

        self.get_logger().info(
            "[localization] ({}, {})\t{} rad\t{} m/s\t{} rad".format(
                position.x, position.y, yaw,
                self.velocity_actual, self.steering_angle_actual
            )
        )

        current_action = np.array(
            [self.acceleration_command, self.steering_angle_actual])
        current_state = np.array([position.x, position.y, yaw, self.velocity_actual])

        # mpc action for current instant
        new_action, self.old_closest_index, mpc_path_size = run_mpc(
            current_action, 
            current_state, 
            self.path,
            self.old_closest_index
        )

        if new_action is None:
            self.get_logger().debug("No action calculated")
            return

        self.acceleration_command = new_action[0]
        self.steering_angle_command = new_action[1]

        # set done
        to_end = [
            position.x - self.path[-1][0],
            position.y - self.path[-1][1]
        ]

        if (np.linalg.norm(to_end) <= P.done_trigger_dist) and \
            (self.old_closest_index == (mpc_path_size - 1)):
            self.done = True
            self.adapter.eufs_mission_finished()


    def path_callback(self, points_list):
        """!
        @brief Path callback.
        @param self The object pointer.
        @param path Path message.
        """
        path = []
        path_speeds = []

        print("Received path\n{}".format(points_list.points))
        print("------------")
        for i, point in enumerate(points_list.points):
            self.get_logger().debug("[received] ({}, {})".format(point.x, point.y))
            path.append([point.x, point.y])

            dist_from_end = len(points_list.points) - i

            # velocity reference is decreased to 0 uniformly after start break pos
            speed = P.VEL*min(1.0, dist_from_end / P.START_BREAKING_POS)
            path_speeds.append(speed)

        self.path = np.array(path)
        self.path_speeds = np.array(path_speeds)


def main(args=None):
    rclpy.init(args=args)

    control_node = ControlNode()

    rclpy.spin(control_node)

    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()