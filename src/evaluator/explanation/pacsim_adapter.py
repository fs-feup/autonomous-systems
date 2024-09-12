from evaluator.adapter import Adapter
import numpy as np
import datetime
import rclpy
import message_filters
from custom_interfaces.msg import ConeArray, VehicleState
from visualization_msgs.msg import MarkerArray
from pacsim.msg import PerceptionDetections
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped
from evaluator.formats import (
    format_vehicle_state_msg,
    format_cone_array_msg,
    format_transform_stamped_msg,
    format_twist_with_covariance_stamped_msg,
    format_marker_array_msg,
)

class PacsimAdapter(Adapter):
    """!
    Adapter class for subscribing to PacSim topics
    """

    def __init__(self, node: rclpy.node.Node):
        """!
        Initializes the PacSim Adapter.

        Args:
            node (Node): ROS2 node instance.
        """

    def state_estimation_callback(
        self,
        vehicle_state: VehicleState,
        map: ConeArray,
    ):
        """!
        Callback function to process synchronized messages
        and compute state estimation metrics.

        Args:
            vehicle_state (VehicleState): Vehicle state estimation data.
            map (ConeArray): Map data.
        """

    def groundtruth_velocity_callback(
        self, groundtruth_velocity: TwistWithCovarianceStamped
    ):
        """!
        Callback function to process ground truth velocity messages.

        Args:
            groundtruth_velocity (TwistWithCovarianceStamped): Ground truth velocity data.
        """

    def groundtruth_map_callback(self, groundtruth_map: MarkerArray):
        """!
        Callback function to process ground truth map messages.
        It also marks the planning's initial timestamp

        Args:
            groundtruth_map (MarkerArray): Ground truth map data.
        """