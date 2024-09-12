from evaluator.adapter import Adapter
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
import datetime
from sensor_msgs.msg import PointCloud2
from evaluator.formats import format_cone_array_msg
from custom_interfaces.msg import ConeArray, VehicleState
from evaluator.formats import (
    format_vehicle_state_msg,
    format_cone_array_msg,
    format_nav_odometry_msg,
)
import rclpy

class FSDSAdapter(Adapter):
    """!
    Adapter class for adapting/synchronizing computations' data with ground truth.
    """

    def __init__(self, node: rclpy.node.Node):
        """!
        Initializes the FSDSAdapter.

        Args:
            node (Node): ROS2 node instance.
            point_cloud_topic (str): Topic for point cloud data.
        """

    def perception_callback(
        self, perception: ConeArray, point_cloud: PointCloud2, odometry: Odometry
    ):
        """!
        Callback function to process synchronized messages and compute perception metrics.

        Args:
            perception (ConeArray): Cone array perception data.
            point_cloud (PointCloud2): Point cloud data.
            odometry (Odometry): Odometry data.
        """

    def odometry_callback(
        self, odometry: Odometry
    ):
        """!
        Callback function to mark the planning's initial timestamp

        Args:
            odometry (Odometry): Behicle's odometry information.
        """

    def state_estimation_callback(
        self, vehicle_state: VehicleState, map: ConeArray, odometry: Odometry
    ):
        """!
        Callback function to process synchronized messages and compute perception metrics.

        Args:
            vehicle_state (VehicleState): Vehicle state estimation message.
            map (ConeArray): Cone array message.
            odometry (Odometry): Odometry message.
        """

    def create_perception_ground_truth(self, odometry: Odometry) -> list:
        """!
        Creates ground truth from odometry data and a predefined track.

        Args:
            odometry (Odometry): Odometry data.

        Returns:
            list: List of ground truth cone positions (relative positions to the car).
        """

    def read_track(self, filename: str) -> None:
        """!
        Reads track data from a CSV file.

        Args:
            filename (str): Path to the CSV file containing track data.
        """

    def parse_track_cone(line: str) -> np.ndarray:
        """!
        Parses a line from the track CSV file to extract cone data.

        Args:
            line (str): A line from the track CSV file.

        Returns:
            numpy.ndarray: Array containing cone position data.
        """

    def quaternion_to_rotation_matrix(quaternion: Quaternion) -> np.ndarray:
        """!
        Converts quaternion to a rotation matrix.

        Args:
            quaternion (Quaternion): Quaternion representing rotation.

        Returns:
            numpy.ndarray: Rotation matrix.
        """