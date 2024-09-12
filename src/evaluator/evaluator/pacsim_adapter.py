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
    Adapter class to manage the use of Pacsim.
    """

    def __init__(self, node: rclpy.node.Node):
        """!
        Initializes the PacSim Adapter.

        Args:
            node (Node): ROS2 node instance.
        """
        super().__init__(node)
        self._groundtruth_velocity_ = None
        self._groundtruth_map_ = None
        self.node.groundtruth_map_subscription_ = self.node.create_subscription(
            MarkerArray,
            "/pacsim/map",
            self.groundtruth_map_callback,
            10,
        )  # because the map gets published only once at the beginning
        self.node.simulated_perception_subscription_ = self.node.create_subscription(
            PerceptionDetections,
            "/pacsim/perception/livox_front/landmarks",
            self.simulated_perception_callback,
            10,
        )
        self.node.groundtruth_velocity_subscription_ = self.node.create_subscription(
            TwistWithCovarianceStamped,
            "/pacsim/velocity",
            self.groundtruth_velocity_callback,
            10,
        )  # Pose is given by transforms

        self._time_sync_ = message_filters.ApproximateTimeSynchronizer(
            [
                self.node.vehicle_state_subscription_,
                self.node.map_subscription_,
            ],
            10,
            0.1,
        )

        self._time_sync_.registerCallback(self.state_estimation_callback)

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
        if self._groundtruth_map_ is None or self._groundtruth_velocity_ is None:
            return
        transform: TransformStamped = self.node.transform_buffer_.lookup_transform(
            "map", "car", rclpy.time.Time()
        )
        if transform is None:
            return
        groundtruth_pose_treated: np.ndarray = format_transform_stamped_msg(transform)
        groundtruth_velocities_treated: np.ndarray = (
            format_twist_with_covariance_stamped_msg(self._groundtruth_velocity_)
        )  # [vx, vy, w]
        map_treated: np.ndarray = format_cone_array_msg(map)
        groundtruth_map_treated: np.ndarray = format_marker_array_msg(
            self._groundtruth_map_
        )
        pose_treated, velocities_treated = format_vehicle_state_msg(
            vehicle_state
        )  # [x, y, yaw], [v, v, w]
        self.node.compute_and_publish_state_estimation(
            pose_treated,
            groundtruth_pose_treated,
            velocities_treated,
            groundtruth_velocities_treated,
            map_treated,
            groundtruth_map_treated,
        )

    def groundtruth_velocity_callback(
        self, groundtruth_velocity: TwistWithCovarianceStamped
    ):
        """!
        Callback function to process ground truth velocity messages.

        Args:
            groundtruth_velocity (TwistWithCovarianceStamped): Ground truth velocity data.
        """
        self._groundtruth_velocity_: TwistWithCovarianceStamped = groundtruth_velocity

    def groundtruth_map_callback(self, groundtruth_map: MarkerArray):
        """!
        Callback function to process ground truth map messages.
        It also marks the planning's initial timestamp

        Args:
            groundtruth_map (MarkerArray): Ground truth map data.
        """
        self._groundtruth_map_: MarkerArray = groundtruth_map
