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
        super().__init__(node)

        # Subscription to odometry messages
        self.node.odometry_subscription_ = message_filters.Subscriber(
            self.node,
            Odometry,
            "/testing_only/odom",
        )

        self.node.odometry_ground_truth_subscription = self.node.create_subscription(
            Odometry,
            "/testing_only/odom",
            self.odometry_callback,
            10,
        )

        # ApproximateTimeSynchronizer for synchronizing perception, point cloud, and odometry messages
        self._perception_sync_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.perception_subscription_,
                self.node.point_cloud_subscription_,
                self.node.odometry_subscription_,
            ],
            10,
            0.1,
        )
        self._state_estimation_sync_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.vehicle_state_subscription_,
                self.node.map_subscription_,
                self.node.odometry_subscription_,
            ],
            10,
            0.1,
        )

        self._perception_sync_.registerCallback(self.perception_callback)
        self._state_estimation_sync_.registerCallback(self.state_estimation_callback)

        # List to store track data
        self.track = []

        # Read track data from a CSV file
        self.read_track("src/evaluator/evaluator/tracks/track_droneport.csv")

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

        perception_ground_truth: np.ndarray = self.create_perception_ground_truth(
            odometry
        )
        perception_output: np.ndarray = format_cone_array_msg(perception)
        self.node.compute_and_publish_perception(
            perception_output, perception_ground_truth
        )
    
    def odometry_callback(
        self, odometry: Odometry
    ):
        """!
        Callback function to mark the planning's initial timestamp

        Args:
            odometry (Odometry): Behicle's odometry information.
        """
        
        if self.node.use_simulated_se_:
            self.node.map_receive_time_ = datetime.datetime.now()

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
        pose_treated, velociies_treated = format_vehicle_state_msg(vehicle_state)
        map_treated: np.ndarray = format_cone_array_msg(map)
        groundtruth_pose_treated: np.ndarray = format_nav_odometry_msg(odometry)
        groundtruth_map_treated: np.ndarray = np.array(self.track)
        empty_groundtruth_velocity_treated = np.array([0, 0, 0])
        self.node.compute_and_publish_state_estimation(
            pose_treated,
            groundtruth_pose_treated,
            velociies_treated,
            empty_groundtruth_velocity_treated,
            map_treated,
            groundtruth_map_treated,
        )

    def create_perception_ground_truth(self, odometry: Odometry) -> list:
        """!
        Creates ground truth from odometry data and a predefined track.

        Args:
            odometry (Odometry): Odometry data.

        Returns:
            list: List of ground truth cone positions (relative positions to the car).
        """
        rotation_matrix = FSDSAdapter.quaternion_to_rotation_matrix(
            odometry.pose.pose.orientation
        )
        perception_ground_truth = []

        for cone in self.track:
            cone_position = np.array([cone[0], cone[1], 0])
            transformed_position = np.dot(rotation_matrix, cone_position) + np.array(
                [odometry.pose.pose.position.x, odometry.pose.pose.position.y, 0]
            )

            perception_ground_truth.append(np.append(transformed_position[:2], 0))

        return np.array(perception_ground_truth)

    def read_track(self, filename: str) -> None:
        """!
        Reads track data from a CSV file.

        Args:
            filename (str): Path to the CSV file containing track data.
        """
        with open(filename, "r") as file:
            for line in file:
                self.track.append(self.parse_track_cone(line.strip()))

    @staticmethod
    def parse_track_cone(line: str) -> np.ndarray:
        """!
        Parses a line from the track CSV file to extract cone data.

        Args:
            line (str): A line from the track CSV file.

        Returns:
            numpy.ndarray: Array containing cone position data.
        """
        cone_color_dictionary: dict[str, int] = {
            "blue": 0,
            "yellow": 1,
            "orange": 2,
            "big_orange": 3,
            "unknown": 4,
        }
        words = line.split(",")
        y = float(words[1])
        x = float(words[2])
        color = cone_color_dictionary[words[0]]
        return np.array([x, y, color, 1])  # 4 for unknown color

    @staticmethod
    def quaternion_to_rotation_matrix(quaternion: Quaternion) -> np.ndarray:
        """!
        Converts quaternion to a rotation matrix.

        Args:
            quaternion (Quaternion): Quaternion representing rotation.

        Returns:
            numpy.ndarray: Rotation matrix.
        """
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z

        # First row of the rotation matrix
        r00 = 1 - 2 * (y**2 + z**2)
        r01 = 2 * (x * y - z * w)
        r02 = 2 * (x * z + y * w)

        # Second row of the rotation matrix
        r10 = 2 * (x * y + z * w)
        r11 = 1 - 2 * (x**2 + z**2)
        r12 = 2 * (y * z - x * w)

        # Third row of the rotation matrix
        r20 = 2 * (x * z - z * w)
        r21 = 2 * (y * z + x * w)
        r22 = 1 - 2 * (x**2 + y**2)

        rot_matrix = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
        return rot_matrix
