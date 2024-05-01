from evaluator.adapter import Adapter
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
import numpy as np
import datetime
from custom_interfaces.msg import ConeArray
import rclpy


class FSDSAdapter(Adapter):
    """!
    Adapter class for adapting/synchronizing computaions' data with ground truth.
    """

    def __init__(self, node: rclpy.node.Node, point_cloud_topic: str):
        """!
        Initializes the FSDSAdapter.

        Args:
            node (Node): ROS2 node instance.
            point_cloud_topic (str): Topic for point cloud data.
        """
        super().__init__(node, point_cloud_topic)

        # Subscription to odometry messages
        self.node.odometry_subscription = message_filters.Subscriber(
            self.node,
            Odometry,
            "/testing_only/odom",
        )

        # ApproximateTimeSynchronizer for synchronizing perception, point cloud, and odometry messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.perception_subscription,
                self.node.point_cloud_subscription,
                self.node.odometry_subscription,
            ],
            10,
            0.1,
        )

        self.ts.registerCallback(self.perception_callback)

        # List to store track data
        self.track = []

        # Read track data from a CSV file
        self.readTrack("src/evaluator/evaluator/tracks/track_droneport.csv")

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

        self.node.perception_ground_truth: list = self.create_ground_truth(odometry)
        self.node.perception_output: list = self.create_perception_output(perception)
        self.node.compute_and_publish_perception()

    def create_perception_output(self, perception: PointCloud2):
        """!
        Creates perception output from cone array messages.

        Args:
            perception (ConeArray): Cone array perception data.

        Returns:
            list: List of perceived cones.
        """
        perception_output = []

        for cone in perception.cone_array:
            perception_output.append(np.array([cone.position.x, cone.position.y, 0.0]))

        return perception_output

    def create_ground_truth(self, odometry: Odometry):
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
            cone_position = np.array([cone[0], cone[1], 0.0])
            transformed_position = np.dot(rotation_matrix, cone_position) + np.array(
                [odometry.pose.pose.position.x, odometry.pose.pose.position.y, 0.0]
            )

            perception_ground_truth.append(f"{transformed_position}")
            self.node.get_logger().info(transformed_position)

        return perception_ground_truth

    def readTrack(self, filename: str):
        """!
        Reads track data from a CSV file.

        Args:
            filename (str): Path to the CSV file containing track data.
        """
        with open(filename, "r") as file:
            for line in file:
                self.track.append(self.parseTrackCone(line.strip()))

    @staticmethod
    def parseTrackCone(line: str):
        """!
        Parses a line from the track CSV file to extract cone data.

        Args:
            line (str): A line from the track CSV file.

        Returns:
            numpy.ndarray: Array containing cone position data.
        """
        words = line.split(",")
        y = float(words[1])
        x = float(words[2])
        return np.array([x, y, 0])

    @staticmethod
    def quaternion_to_rotation_matrix(quaternion: Quaternion):
        """!
        Converts quaternion to a rotation matrix.

        Args:
            quaternion (Quaternion): Quaternion representing rotation.

        Returns:
            numpy.ndarray: Rotation matrix.
        """
        q0, q1, q2, q3 = quaternion.x, quaternion.y, quaternion.z, quaternion.w

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        rot_matrix = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
        return rot_matrix
