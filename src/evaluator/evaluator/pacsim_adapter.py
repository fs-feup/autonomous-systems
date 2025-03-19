from evaluator.adapter import Adapter
import numpy as np
import datetime
import rclpy
import message_filters
from custom_interfaces.msg import ConeArray, PathPointArray, Pose, Velocities
from visualization_msgs.msg import MarkerArray
from pacsim.msg import PerceptionDetections
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped
from evaluator.formats import (
    format_vehicle_pose_msg,
    format_velocities_msg,
    format_cone_array_msg,
    format_transform_stamped_msg,
    format_twist_with_covariance_stamped_msg,
    format_marker_array_msg,
    format_path_point_array_msg,
    get_blue_and_yellow_cones_after_msg_treatment,
)


class PacsimAdapter(Adapter):
    """!
    Adapter class to manage the use of Pacsim.
    """

    def __init__(self, node: rclpy.node.Node):
        """!
        Initializes the PacSim Adapter.

        Args:
            node (Node): ROS2 node instance.
        """
        super().__init__(node)
        self._groundtruth_map_ = None
        self.node.groundtruth_map_subscription_ = self.node.create_subscription(
            MarkerArray,
            "/pacsim/map",
            self.groundtruth_map_callback,
            10,
        )  # because the map gets published only once at the beginning

        self._time_sync_velocities_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.velocities_subscription_,
                self.node.groundtruth_velocity_subscription_,
            ],
            10,
            0.02,
        )

        self._time_sync_slam_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.vehicle_pose_subscription_,
                self.node.map_subscription_,
            ],
            10,
            0.1,
        )

        self._time_sync_slam_.registerCallback(self.slam_callback)

        self._time_sync_velocities_.registerCallback(self.velocities_callback)

        self._planning_time_sync_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.planning_subscription_,
                self.node.planning_gt_subscription_,
            ],
            10,
            0.5,
        )

        self._planning_time_sync_.registerCallback(self.planning_callback)

    def slam_callback(
        self,
        vehicle_pose: Pose,
        map: ConeArray,
    ):
        """!
        Callback function to process synchronized messages
        and compute pose and map metrics.

        Args:
            vehicle_pose (Pose): Pose of the vehicle.
            map (ConeArray): Map data.
        """
        if self._groundtruth_map_ is None:
            self.node.get_logger().warn("Groundtruth map not received")
            return
        transform: TransformStamped = self.node.transform_buffer_.lookup_transform(
            "map", "car", rclpy.time.Time()
        )
        if transform is None:
            return
        groundtruth_pose_treated: np.ndarray = format_transform_stamped_msg(transform)
        map_treated: np.ndarray = format_cone_array_msg(map)
        groundtruth_map_treated: np.ndarray = format_marker_array_msg(
            self._groundtruth_map_
        )
        pose_treated: np.ndarray = format_vehicle_pose_msg(vehicle_pose)
        self.node.compute_and_publish_slam(
            pose_treated,
            groundtruth_pose_treated,
            map_treated,
            groundtruth_map_treated,
        )

    def velocities_callback(
        self,
        velocities: Velocities,
        groundtruth_velocity: TwistWithCovarianceStamped,
    ):
        """!
        Callback function to process synchronized messages
        and compute state estimation metrics.

        Args:
            velocities (Velocities): Vehicle velocities estimation data.
            groundtruth_velocity (TwistWithCovarianceStamped): Ground truth velocity data.
        """

        groundtruth_velocities_treated: np.ndarray = (
            format_twist_with_covariance_stamped_msg(groundtruth_velocity)
        )  # [vx, vy, w]
        velocities_treated = format_velocities_msg(velocities)  # [x, y, yaw], [v, v, w]
        self.node.compute_and_publish_velocities(
            velocities_treated,
            groundtruth_velocities_treated,
        )

    def planning_callback(
        self,
        planning_output: PathPointArray,
        path_ground_truth: PathPointArray,
    ):
        """!
        Callback function to process synchronized messages and compute planning metrics.

        Args:
            planning_output (PathPointArray): Planning output.
            path_ground_truth (PathPointArray): Groundtruth path message.
        """
        if self._groundtruth_map_ is None:
            return
        map_ground_truth_treated: np.ndarray = format_marker_array_msg(
            self._groundtruth_map_
        )
        blue_cones, yellow_cones = get_blue_and_yellow_cones_after_msg_treatment(
            map_ground_truth_treated
        )
        planning_output_treated: np.ndarray = format_path_point_array_msg(
            planning_output
        )
        path_ground_truth_treated: np.ndarray = format_path_point_array_msg(
            path_ground_truth
        )
        self.node.compute_and_publish_planning(
            planning_output_treated,
            path_ground_truth_treated,
            blue_cones,
            yellow_cones,
        )

    def groundtruth_map_callback(self, groundtruth_map: MarkerArray):
        """!
        Callback function to process ground truth map messages.
        It also marks the planning's initial timestamp

        Args:
            groundtruth_map (MarkerArray): Ground truth map data.
        """
        self._groundtruth_map_: MarkerArray = groundtruth_map
