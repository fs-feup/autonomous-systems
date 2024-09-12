from evaluator.adapter import Adapter
from eufs_msgs.msg import ConeArrayWithCovariance, CarState
from nav_msgs.msg import Odometry
from custom_interfaces.msg import ConeArray, VehicleState
import rclpy
import datetime
import message_filters
import numpy as np
from evaluator.formats import (
    format_cone_array_msg,
    format_vehicle_state_msg,
    format_eufs_cone_array_with_covariance_msg,
    format_nav_odometry_msg,
    format_car_state_msg,
)

class EufsAdapter(Adapter):
    """!
    Adapter class for subscribing to EUFS topics
    """

    def __init__(self, node: rclpy.node.Node):
        """!
        Initializes the EUFS Adapter.

        Args:
            node (Node): ROS2 node instance.
        """

    def simulated_vehicle_state_callback(self, msg: CarState):
        """!
        Callback function to mark the initial timestamp of the control execution

        Args:
            msg (CarState): Car state coming from EUFS simulator
        """

    def state_estimation_callback(
        self,
        vehicle_state: VehicleState,
        map: ConeArray,
    ):
        """!
        Callback function to process synchronized messages and compute state estimation metrics.

        Args:
            vehicle_state (VehicleState): Vehicle state estimation message.
            map (ConeArray): Cone array message.
        """

    def perception_callback(
        self, perception_output: ConeArray, ground_truth: ConeArrayWithCovariance
    ):
        """!
        Callback function to process synchronized messages and compute perception metrics.

        Args:
            perception_output (ConeArray): Perception Output.
        """

    def groundtruth_map_callback(self, track: ConeArrayWithCovariance):
        """!
        Callback function to process groundtruth map messages.

        Args:
            track (ConeArrayWithCovariance): Groundtruth track data.
        """

    def groundtruth_vehicle_state_callback(self, vehicle_state: Odometry):
        """!
        Callback function to process groundtruth vehicle_state messages.

        Args:
            vehicle_state (Odometry): Groundtruth vehicle_state data.
        """